const RosClient = require("roslib");

module.exports = {
    ROSController: class {

        constructor(url) {
            this.url = url;
            this.ros = null;
            this.cmdVel = {
                topic: null,
                data: null,
                velocities: {
                    linear: 0.0,
                    angular: 0.0,
                    mLinear: 0.5,
                    mAngular: 1.0,

                    clampVelocities() {
                        Math.min(
                            Math.max(this.linear, -this.mLinear),
                            this.mLinear
                        );

                        Math.min(
                            Math.max(this.angular, -this.mAngular),
                            this.mAngular
                        );
                    }
                }
            };

            this.odom = {
                topic: null,
                coordinates: {
                    x: "0.000",
                    y: "0.000"
                }
            };

            this.amcl = {
                topic: null,
                coordinates: {
                    x: 0.00,
                    y: 0.00
                }
            };

            this.map = {
                topic: null,
                mapData: null
            }

            this.initRos();
            this.createTopics();
        }

        initRos() {

            this.ros = new RosClient.Ros({
                encoding: 'ASCII',
                url: this.url
            });

            this.ros.on('connection', function () {
                console.log('Connected to websocket server.');
            });

            this.ros.on('error', function (error) {
                console.log('Error connecting to websocket server: ', error);
                process.exit();
            });

            this.ros.on('close', function () {
                console.log('Connection to websocket server closed.');
                process.exit();
            });
        }

        createTopics() {
            this.cmdVel.topic = new RosClient.Topic({
                ros: this.ros,
                name: '/rabbitamr_diff_drive_controller/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            });

            this.odom.topic = new RosClient.Topic({
                ros: this.ros,
                name: '/odom',
                messageType: 'nav_msgs/Odometry'
            });

            this.amcl.topic = new RosClient.Topic({
                ros: this.ros,
                name: '/amcl_pose',
                messageType: 'geometry_msgs/PoseWithCovarianceStamped'
            });

            this.map.topic = new RosClient.Topic({
                ros: this.ros,
                name: '/map',
                messageType: 'nav_msgs/OccupancyGrid'
            });

            this.odom.topic.subscribe(message => this.odomCallback(message));
            this.amcl.topic.subscribe(message => this.amclCallback(message));
            this.map.topic.subscribe(message => this.mapCallback(message));
        }

        async odomCallback(message) {
            this.odom.coordinates.x = (message.pose.pose.position.x).toString().substring(0, 5);
            this.odom.coordinates.y = (message.pose.pose.position.y).toString().substring(0, 5);
        }

        async amclCallback(message) {
            this.amcl.coordinates.x = message.pose.pose.position.x;
            this.amcl.coordinates.y = message.pose.pose.position.y;
        }

        async mapCallback(message) {
            this.map.mapData = {
                width: message.info.width,
                height: message.info.height,
                resolution: message.info.resolution,
                origin: message.info.origin,
                data: message.data
            }

            this.map.topic.unsubscribe();
        }

        async setCmd(req, res, next) {
            res.setHeader('Content-Type', 'application/json');
            let ok = true;

            this.cmdVel.velocities.linear = req.body.linear;
            this.cmdVel.velocities.angular = req.body.angular;

            res.send("{\"status\": \"forward\"}");

            if (ok) {
                this.cmdVel.velocities.clampVelocities();

                this.cmdVel.topic.publish(
                    new RosClient.Message({
                        linear: {
                            x: parseFloat(this.cmdVel.velocities.linear),
                            y: 0.0,
                            z: 0.0
                        },
                        angular: {
                            x: 0.0,
                            y: 0.0,
                            z: parseFloat(this.cmdVel.velocities.angular)
                        }
                    })
                );
            }
        }

        async getInfo(req, res, next) {
            res.setHeader('Content-Type', 'application/json');
            res.send(this.odom.coordinates);
        }

        async getAmcl(req, res, next) {
            res.setHeader('Content-Type', 'application/json');
            res.send(this.amcl.coordinates);
        }

        async getMap(req, res, next) {
            res.setHeader('Content-Type', 'application/json');
            res.send(this.map.mapData);
        }

        async setLandmark(req, res, next) {
            res.setHeader('Content-Type', 'application/json');

        }

        async saveCoords(req, res, next) {
            res.setHeader('Content-Type', 'application/json');

            const actionClient = new ROSLIB.ActionClient({
                ros: ros,
                serverName: '/set_landmark',
                actionName: 'set_landmark/LandmarkAction'
            });

            const goal = new ROSLIB.Goal({
                actionClient: actionClient,
                goalMessage: {
                    time_now: new Date()
                }
            });

            goal.on('feedback', function (feedback) {
                console.log(feedback);
            });

            goal.on('result', function (result) {
                if (result["saved_landmark"]) {
                    //draw landmark
                }
            });

            goal.send();
        }
    }
};
