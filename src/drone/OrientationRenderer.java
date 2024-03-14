package drone;

import javafx.animation.RotateTransition;
import javafx.application.Application;
import javafx.geometry.Point3D;
import javafx.stage.Stage;
import javafx.util.Duration;
import javafx.scene.Scene;
import javafx.scene.Camera;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SubScene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.DrawMode;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import math.Quaternion;
import pose.Orientation;

import java.io.IOException;
import java.lang.Math;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.Random;

public class OrientationRenderer extends Application{
    //rotational offset of the camera object, used to render the block in a better angle
    private int rotateY = -20;
    private int rotateX = -20;

    private volatile static Quaternion q = Quaternion.IDENTITY;

    @Override
    public void start(Stage primaryStage) throws Exception {
        primaryStage.setResizable(false);
        // instantiates the block
        Box block = new Box(7, 1, 5);
        block.setMaterial(new PhongMaterial(Color.RED));
        block.setDrawMode(DrawMode.LINE);

        //creates the lines marking the x, y, and z axis
        Box lineX = new Box(100, 0.01, 0.01);
        Box lineY = new Box(0.01, 100, 0.01);
        Box lineZ = new Box(0.01, 0.01, 100);

        // Create and position camera
        Camera camera = new PerspectiveCamera(true);
        camera.getTransforms().addAll (
                new Rotate(rotateY, Rotate.Y_AXIS),
                new Rotate(rotateX, Rotate.X_AXIS),
                new Translate(0, 0, -15));

        // Build the Scene Graph
        Group root = new Group();
        root.getChildren().add(camera);
        root.getChildren().add(block);
        root.getChildren().add(lineX);
        root.getChildren().add(lineY);
        root.getChildren().add(lineZ);

        //sets up the camera and window size properly
        SubScene subScene = new SubScene(root, 1280, 720); //window size, adjust numbers if necessary
        subScene.setFill(Color.ALICEBLUE);
        subScene.setCamera(camera);
        Group group = new Group();
        group.getChildren().add(subScene);
        Scene scene = new Scene(group);

        //displays the scene with all the objects in it on screen
        primaryStage.setScene(scene);
        primaryStage.show();

        //creates an infinite loop to get the updated rotation of the block and render it out
        RotateTransition rt = new RotateTransition(Duration.millis(40), block); //The refresh rate of the loop is set in the constructor, can be changed to make it loop faster or slower
        rt.setCycleCount(1);
        rt.play();
        rt.setOnFinished(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                /*
                 * TODO: replace the getPos() function with a function that gets the quaternion
                 * as a double array with 4 indexes and the w, i, j, and k values stored
                 * in the indexes 0-3.
                 */
                double[] vec = getPos();

                //math to calculate the angle of rotation and the axis of rotation from the given quaternion
                double theta, Ax, Ay, Az, angle;
                theta = Math.acos(vec[0]);
                Ax = -vec[1] / Math.sin(theta);
                Az = -vec[2] / Math.sin(theta);
                Ay = -vec[3] / Math.sin(theta);
                angle = Math.toDegrees(2 * theta);

                //renders out the new rotation of the block
                block.getTransforms().clear();
                block.getTransforms().add(new Rotate(angle, new Point3D(Ax, Ay, Az)));
                rt.play(); //loops the transition again
            }
        });
    }

    //for testing, spits out a couple random rotations for the rendering to be updated
    public double[] getPos() {
       var q = OrientationRenderer.q;
       return new double[]{q.x0(), q.x1(), q.x2(), q.x3()};
    }


    public static void main(String[] args) throws IOException {
        Thread.startVirtualThread(() -> {
            try (var socket = DatagramChannel.open().bind(new InetSocketAddress("0.0.0.0", 4444))) {
                var bytes = new byte[4 * 8];

                while (!Thread.interrupted()) {
                    socket.receive(ByteBuffer.wrap(bytes));
                    q = Quaternion.ofBytes(bytes);
                }

            } catch (Throwable e) {
                throw new RuntimeException(e);
            }
        });

        launch(args);
    }

}