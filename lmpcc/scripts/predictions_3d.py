#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf_conversions
from cv_msgs.msg import PredictedMoGTracks
from dynamic_reconfigure.server import Server
from ros_intent_slds.cfg import PredictionConverterConfig
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PredictionConverter:
    def __init__(self, output_path_topic='predicted_path_marker'):
        self.show_mog_mean = True
        self.show_mog_covariance = True
        self.show_collapsed_mean = True
        self.show_collapsed_covariance = True
        self.covariance_timestep = 16
        self.path_pub = rospy.Publisher(output_path_topic, MarkerArray, queue_size=10)
        self.cmap = plt.cm.get_cmap('RdYlGn')
        self.config_srv = Server(PredictionConverterConfig, self.config_callback)

    @staticmethod
    def get_pick_color(_id):
        r = float(int(_id * 11 * 32) % 256) / 256
        g = float(int(_id * 7 * 16) % 256) / 256
        b = float(int(_id * 5 * 8) % 256) / 256
        return r, g, b

    # create line strip marker for each unique track id
    def create_marker_array(self, track_data):
        msg_id = 0  # msg marker id, must be unique, not to be confused with unique track id
        marker_array = MarkerArray()
        # create a new marker consisting of multiple points/line strips
        marker = self.create_prediction_path(msg_id, track_data)
        msg_id = msg_id + 1
        marker_array.markers.append(marker)

        track_colors = [0.001, 0.999]  # colors for the two MOGs. bending=red, straight = green
        for track in track_data.tracks:
            combined_color = np.zeros([4])
            score = track.track[self.covariance_timestep - 1].score
            for k in range(len(score)):
                colors_rgba = np.array(self.cmap(track_colors[k]))

                combined_color += colors_rgba * score[k]
                if self.show_mog_covariance:
                    pose = track.track[self.covariance_timestep - 1].pose[k]
                    marker = self.create_covariance_marker(track_data.header, msg_id, pose)
                    certainty = 2 * np.abs(0.5 - score[k])
                    certainty_color = colors_rgba * certainty + (1 - certainty) / 2
                    if self.show_collapsed_covariance:
                        certainty_color[3] = 0.7
                    marker.color = self.make_color_msg(certainty_color)
                    marker_array.markers.append(marker)
                    msg_id = msg_id + 1

            if self.show_collapsed_covariance:
                pose = track.track[self.covariance_timestep - 1].collapsed_pose
                marker = self.create_covariance_marker(track_data.header, msg_id, pose)

                if len(score) > 0:
                    certainty = 2 * np.abs(0.5 - score[0])
                    certainty_color = combined_color * certainty + (1 - certainty) / 2
                    certainty_color[3] = 0.7
                else:
                    certainty_color = [0.7, 0.7, 0.1]
                marker.color = self.make_color_msg(certainty_color)
                marker_array.markers.append(marker)
                msg_id = msg_id + 1
        # Return the entire collection of tracks accumulated up until this point in time
        return marker_array

    def create_prediction_path(self, msg_id, track_data):
        marker = self.init_marker(track_data.header, Marker.POINTS, msg_id)
        # set scale to a visually appealing value
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.00001
        # set orientation to identity
        marker.pose.orientation.w = 1
        track_colors = [0.001, 0.999]  # colors for the two MOGs. bending=red, straight = green
        # loop over all unique track id's
        for track in track_data.tracks:
            for mog in track.track:
                combined_color = np.zeros([4])
                for k in range(len(mog.score)):
                    # assign color based on unique track id
                    # r, g, b = self.get_pick_color(track.id)
                    colors_rgba = np.array(self.cmap(track_colors[k]))
                    combined_color += colors_rgba * mog.score[k]
                    if self.show_mog_mean:
                        marker.points.append(mog.pose[k].pose.position)
                        certainty = 2 * np.abs(0.5 - mog.score[k])
                        certainty_color = colors_rgba * certainty + (1 - certainty) / 2
                        certainty_color[3] = 0.7
                        color = self.make_color_msg(certainty_color)
                        marker.colors.append(color)

                if self.show_collapsed_mean:
                    marker.points.append(mog.collapsed_pose.pose.position)
                    if len(mog.score) > 0:
                        certainty = 2 * np.abs(0.5 - mog.score[0])
                        certainty_color = combined_color * certainty + (1 - certainty) / 2
                        certainty_color[3] = 0.7
                    else:
                        certainty_color = [0.7, 0.7, 0.1]
                    color = self.make_color_msg(certainty_color)
                    marker.colors.append(color)
        return marker

    def init_marker(self, header, marker_type, msg_id):
        marker = Marker()
        marker.header = header
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(secs=1)
        marker.type = marker_type
        marker.id = msg_id
        return marker

    def create_covariance_marker(self, header, msg_id, pose):
        marker = self.init_marker(header, Marker.LINE_STRIP, msg_id)
        covmat = np.reshape(pose.covariance, [6, 6])
        # see https://answers.ros.org/question/11081/plot-a-gaussian-3d-representation-with-markers-in-rviz/
        # except it turns out you don't have to do the quaternion bit yourself
        (eigValues, eigVectors) = np.linalg.eig(covmat[:3, :3])
        transform_mat = np.eye(4, 4)  # create 4x4 transformation matrix
        rotmat = np.vstack(eigVectors)
        transform_mat[:3, :3] = rotmat
        quat = tf_conversions.transformations.quaternion_from_matrix(transform_mat)
        marker.pose.orientation.x = 0#quat[0]
        marker.pose.orientation.y = 0#quat[1]
        marker.pose.orientation.z = 0#quat[2]
        marker.pose.orientation.w = 1#quat[3]
        marker.scale.x = 2 * np.sqrt(eigValues[0])
        marker.scale.y = 2 * np.sqrt(eigValues[1])
        marker.scale.z = 0.0001  # np.sqrt(eigValues[2])
        orientation = marker.pose.orientation
        euler_angle = tf_conversions.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        draw_points = 100
        for angle in range(draw_points + 1): #add one extra point to close the circle
            angle = angle / float(draw_points) * 2 * np.pi
            point = tf_conversions.Point()
            xy = np.array([np.cos(angle) * marker.scale.x,
                           np.sin(angle) * marker.scale.y])

            xy = np.dot(rotmat[:2, :2], xy)
            point.x = pose.pose.position.x + xy[0]
            point.y = pose.pose.position.y + xy[1]
            marker.points.append(point)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        #marker.pose.position = pose.pose.position
        return marker

    def make_color_msg(self, rgba):
        color = ColorRGBA()
        color.r = rgba[0]
        color.g = rgba[1]
        color.b = rgba[2]
        color.a = 1 if len(rgba) < 4 else rgba[3]
        return color

    def callback(self, track_data):
        # create visualization marker of line segments of the separate tracks
        path_msg = self.create_marker_array(track_data)

        # publish visualization of state and path on separate topics
        self.path_pub.publish(path_msg)

    def config_callback(self, config, level):
        self.show_mog_mean = config["show_mog_mean"]
        self.show_mog_covariance = config["show_mog_covariance"]
        self.show_collapsed_mean = config["show_collapsed_mean"]
        self.show_collapsed_covariance = config["show_collapsed_covariance"]
        self.covariance_timestep = config["covariance_timestep"]
        return config


def main():
    rospy.init_node('predictions_3d')

    input_det_topic = rospy.get_param('~input_det_topic', '/future_obj_track')
    output_path_topic = rospy.get_param('~output_path_topic', 'predicted_path')

    tc = PredictionConverter(output_path_topic=output_path_topic)

    det_sub = rospy.Subscriber(input_det_topic, PredictedMoGTracks, tc.callback)

    rospy.loginfo('Subscribed to prediction topic ' + input_det_topic)
    rospy.loginfo('Publishing to prediction topic ' + output_path_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
