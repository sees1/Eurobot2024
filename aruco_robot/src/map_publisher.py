import rospy
from aruco_opencv_msgs.msg import ArucoDetection
from aruco_opencv_msgs.srv import MarkerSize, MarkerSizeResponse
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import numpy as np
import tf.transformations as tr
import tf, tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped
import roslaunch
import subprocess
import os, systemd


class MapPublisher():
    def __init__(self):
        rospy.init_node('map_publisher')

        self.br = tf2_ros.StaticTransformBroadcaster()

        self.sub = rospy.Subscriber("aruco_detections", ArucoDetection, self.arucoCallback)

        self.start_time = rospy.Time.now()    
        self.scan_time = rospy.get_param("~scan_time", 5)
        self.marker_size = rospy.get_param("/map_marker_size", 0.1)

        rospy.wait_for_service('set_marker_size')
        try:
            set_marker_size = rospy.ServiceProxy('set_marker_size', MarkerSize)
            resp1 = set_marker_size(self.marker_size)
            rospy.loginfo(resp1.response)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

        rospy.loginfo(f"Starting map calibration... wait {self.scan_time} sec")
        self.scan_time = rospy.Duration(self.scan_time)

        self.markers_20 = []
        self.markers_23 = []
        self.markers_22 = []


    def arucoCallback(self, data):

        for marker in data.markers:
            if marker.marker_id == 20:
                self.markers_20.append(marker)
            elif marker.marker_id == 22:
                self.markers_22.append(marker)
            elif marker.marker_id == 23:
                self.markers_23.append(marker)

        dt = rospy.Time.now() - self.start_time
        if dt >= self.scan_time and (len(self.markers_20)>0 and len(self.markers_22)>0 and len(self.markers_23)>0):
            rospy.loginfo(f"Found: id20-{len(self.markers_20)} id22-{len(self.markers_22)} id23-{len(self.markers_23)} in {dt.to_sec()} sec")
            self.calcTfMap(self.markers_20, self.markers_22, self.markers_23)
        elif dt >= self.scan_time:
            rospy.logwarn(f"Can't collect enough markers in {dt.to_sec()} sec")

    def calcTfMap(self, markers_20, markers_22, markers_23):
        id20, id22, id23 = ransac_plane_fit(markers_20, markers_22, markers_23)

        v20 = marker_to_point(markers_20[id20])
        v22 = marker_to_point(markers_22[id22])
        v23 = marker_to_point(markers_23[id23])

        a_v = v23-v22
        a_v = a_v/np.linalg.norm(a_v)
        b_v = v20-v22
        b_v = b_v/np.linalg.norm(b_v)
        z_v = np.cross(a_v, b_v)
        z_v = z_v/np.linalg.norm(z_v)

        y_v = np.cross(z_v, a_v)
        y_v = y_v/np.linalg.norm(y_v)

        R = np.array([a_v, y_v, z_v]).T
        R = np.pad(R, (0,1), mode='constant')
        R[3,3] = 1
        q = tr.quaternion_from_matrix(R)

        # mapOffset = (-0.75, -0.5)

        mapOffset = (0,0)

        tf_marker_22_map = TransformStamped()
        tf_marker_22_map.header.stamp = rospy.Time.now()
        tf_marker_22_map.header.frame_id = "brio"
        tf_marker_22_map.child_frame_id = "marker_22_map"

        tf_marker_22_map.transform.translation.x = v22[0]
        tf_marker_22_map.transform.translation.y = v22[1]
        tf_marker_22_map.transform.translation.z = v22[2]

        tf_marker_22_map.transform.rotation.x = q[0]
        tf_marker_22_map.transform.rotation.y = q[1]
        tf_marker_22_map.transform.rotation.z = q[2]
        tf_marker_22_map.transform.rotation.w = q[3]

        tf_map = TransformStamped()
        tf_map.header.stamp = rospy.Time.now()
        tf_map.header.frame_id = "marker_22_map"
        tf_map.child_frame_id = "map"

        tf_map.transform.translation.x = mapOffset[0]
        tf_map.transform.translation.y = mapOffset[1]
        tf_map.transform.translation.z = 0

        tf_map.transform.rotation.x = 0
        tf_map.transform.rotation.y = 0
        tf_map.transform.rotation.z = 0
        tf_map.transform.rotation.w = 1

        self.br.sendTransform([tf_marker_22_map, tf_map])
        self.sub.unregister()

        rospy.loginfo("Map has been calibrated")

        rospy.Service('map_is_callibrated', Empty, None)
        
# Выбор случайных точек из облака точек
def random_marker(markers_list):
    markers = np.array(markers_list)
    idx = np.random.choice(markers.shape[0], 1, replace=False)
    return markers[idx[0]], idx[0]

def fit_plane(points):
    p1, p2, p3 = points
    # Вычисляем нормаль к плоскости как векторное произведение двух векторов на плоскости
    normal = np.cross(p3 - p1, p2 - p1)
    # Вычисляем D для уравнения плоскости
    d = -normal.dot(p1)
    return np.append(normal, d)

def marker_to_point(marker):
    return np.array([marker.pose.position.x,marker.pose.position.y, marker.pose.position.z])

# Расчет расстояния от точки до плоскости
def distance_to_plane(plane, point):
    a, b, c, d = plane
    # Расстояние от точки до плоскости
    return abs(a*point[0] + b*point[1] + c*point[2] + d) / np.linalg.norm([a, b, c])

# Основная функция RANSAC для поиска наилучшей подгонки плоскости к облаку точек
def ransac_plane_fit(markers_20, markers_22, markers_23, n=1000, threshold=0.05):
    best_id20 = None
    best_id22 = None
    best_id23 = None
    best_inliers = -1  # Количество точек, близко расположенных к лучшей плоскости
    
    points = []
    for m in markers_20:
        points.append(marker_to_point(m))
    for m in markers_22:
        points.append(marker_to_point(m))
    for m in markers_23:
        points.append(marker_to_point(m))

    # Повторяем процесс n раз
    for _ in range(n):
        # Выбираем 2 случайные точки
        sample_m_20, id20 = random_marker(markers_20)
        sample_m_22, id22 = random_marker(markers_22)
        sample_m_23, id23 = random_marker(markers_23)

        sample_20 = marker_to_point(sample_m_20)
        sample_22 = marker_to_point(sample_m_22)
        sample_23 = marker_to_point(sample_m_23)

        sample_points = [sample_20, sample_22, sample_23]
        # Получаем уравнение плоскости для этих точек
        plane = fit_plane(sample_points)
        # Считаем, сколько точек лежит близко к данной плоскости
        inliers = sum(1 for point in points if distance_to_plane(plane, point) < threshold)
        
        # Если текущая плоскость лучше предыдущей лучшей, обновляем лучшую плоскость
        if inliers > best_inliers:
            best_inliers = inliers
            # best_plane = plane
            best_id20, best_id22, best_id23 = id20, id22, id23
    
    return best_id20, best_id22, best_id23

if __name__ == '__main__':
    try:
        mp = MapPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass