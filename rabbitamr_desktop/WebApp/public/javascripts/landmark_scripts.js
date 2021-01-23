

function updateMap() {
    // Create the main viewer.
    let viewer = new ROS2D.Viewer({
        divID: 'map',
        width: 400,
        height: 500
    });

    // Setup the map client.
    let gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene
    });

    // Scale the canvas to fit to the map
    gridClient.on('change', function () {
        viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        displayPoseMarker();
    });

    // Show the pose on the map
    function displayPoseMarker() {

        // Create a marker representing the robot.
        let robotMarker = new ROS2D.NavigationArrow({
            size: 10,
            strokeSize: 1,
            fillColor: createjs.Graphics.getRGB(76, 255, 56, 1),
            pulse: true
        });

        // Add the marker to the 2D scene.
        gridClient.rootObject.addChild(robotMarker);
        let initScaleSet = false;

        // Subscribe to the robot's pose updates.
        let poseListener = new ROSLIB.Topic({
            ros: ros,
            name: '/amcl_pose',
            messageType: 'geometry_msgs/PoseWithCovarianceStamped',
            throttle_rate: 100
        });

        poseListener.subscribe(function (pose) {

            // Orientate the marker based on the robot's pose.
            robotMarker.x = pose.pose.pose.position.x;
            coordinates.x = pose.pose.pose.position.x;
            robotMarker.y = -pose.pose.pose.position.y;
            coordinates.y = -pose.pose.pose.position.y;

            if (!initScaleSet) {
                robotMarker.scaleX = 1.0 / viewer.scene.scaleX;
                robotMarker.scaleY = 1.0 / viewer.scene.scaleY;
                initScaleSet = true;
            }

            robotMarker.rotation = viewer.scene.rosQuaternionToGlobalTheta(pose.pose.pose.orientation);

        });
    }


}