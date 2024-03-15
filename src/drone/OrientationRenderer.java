package drone;

import javafx.animation.RotateTransition;
import javafx.application.Application;
import javafx.geometry.Point3D;
import javafx.scene.shape.Line;
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
import math.Vector3;
import pose.Orientation;

import java.io.IOException;
import java.lang.Math;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.util.Arrays;
import java.util.Random;

import static java.lang.Math.PI;
import static math.Vector3.I;

public class OrientationRenderer extends Application {
    //rotational offset of the camera object, used to render the block in a better angle
    private int rotateY = -20;
    private int rotateX = -20;

    private volatile static Quaternion q = Quaternion.IDENTITY;
    private volatile static Vector3 v = Vector3.K;

    @Override
    public void start(Stage primaryStage) throws Exception {
        primaryStage.setResizable(false);
        // instantiates the block
        Box block = new Box(7, 1, 5);
        block.setMaterial(new PhongMaterial(Color.RED));
        block.setDrawMode(DrawMode.LINE);

        var pointer = new Box(0.1, 0.1, 2);
        pointer.setDrawMode(DrawMode.FILL);

        //creates the lines marking the x, y, and z axis
        Box lineX = new Box(100, 0.01, 0.01);
        Box lineY = new Box(0.01, 100, 0.01);
        Box lineZ = new Box(0.01, 0.01, 100);

        // Create and position camera
        Camera camera = new PerspectiveCamera(true);
        camera.getTransforms().addAll(
                new Rotate(rotateY, Rotate.Y_AXIS),
                new Rotate(rotateX, Rotate.X_AXIS),
                new Translate(0, 0, -15));

        // Build the Scene Graph
        Group root = new Group();
        root.getChildren().add(camera);
        root.getChildren().add(block);
//        root.getChildren().add(pointer);
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
        rt.setOnFinished(event -> {
            /*
             * TODO: replace the getPos() function with a function that gets the quaternion
             * as a double array with 4 indexes and the w, i, j, and k values stored
             * in the indexes 0-3.
             */
            double[] vec = getPos();

            //math to calculate the angle of rotation and the axis of rotation from the given quaternion
            AxisAngle result = getAxisAngle(vec);

            //renders out the new rotation of the block
            block.getTransforms().clear();
            block.getTransforms().add(new Rotate(result.angle(), new Point3D(result.Ax(), result.Ay(), result.Az())));

            var vector = I.scale(0.5).add(v.scale(0.5)).normalized();
            pointer.getTransforms().clear();
            pointer.getTransforms().add(new Rotate(180, new Point3D(vector.x(), vector.y(), vector.z())));

            rt.play(); //loops the transition again
        });
    }

    private static AxisAngle getAxisAngle(double[] vec) {
        double theta, Ax, Ay, Az, angle;
        theta = Math.acos(vec[0]);
        Ax = -vec[1] / Math.sin(theta);
        Az = -vec[2] / Math.sin(theta);
        Ay = -vec[3] / Math.sin(theta);
        angle = Math.toDegrees(2 * theta);
        AxisAngle result = new AxisAngle(Ax, Ay, Az, angle);
        return result;
    }

    private record AxisAngle(double Ax, double Ay, double Az, double angle) {
    }

    //for testing, spits out a couple random rotations for the rendering to be updated
    public double[] getPos() {
        var q = OrientationRenderer.q;
        return new double[]{q.x0(), q.x1(), q.x2(), q.x3()};
    }


    public static void main(String[] args) throws IOException {
        Thread.startVirtualThread(() -> {
            try (var socket = DatagramChannel.open().bind(new InetSocketAddress("0.0.0.0", 4444))) {
                var bytes = new byte[4 * 8 + 3 * 8];

                while (!Thread.interrupted()) {
                    var bb = ByteBuffer.wrap(bytes);
                    socket.receive(bb);
                    bb.flip();

                    q = Quaternion.ofBytes(bytes);
                    if (bb.remaining() >= 4 * 8 + 3 * 8)
                        v = Vector3.ofBytes(Arrays.copyOfRange(bytes, 4 * 8, 4 * 8 + 3 * 8));
                }

            } catch (Throwable e) {
                throw new RuntimeException(e);
            }
        });

        launch(args);
    }

}