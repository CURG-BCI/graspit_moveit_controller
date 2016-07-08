import moveit_commander
import rospy


class ExtendedPlanningSceneInterface(moveit_commander.PlanningSceneInterface):
    def __init__(self):
        moveit_commander.PlanningSceneInterface.__init__(self)

    def add_scaled_mesh(self, name, pose, filename, scale):
        collision_object = self._PlanningSceneInterface__make_mesh(name, pose, filename)
        self.__scale_mesh(collision_object, scale)
        self._pub_co.publish(collision_object)
        rospy.loginfo(self.__class__.__name__ + '::add_scaled_mesh::' +
                     'Sent mesh name: %s filename $s'%(name, filename))

    def __scale_mesh(self, collision_object, scale):
        for point in collision_object.meshes[0].vertices:
            point.x *= scale[0]
            point.y *= scale[1]
            point.z *= scale[2]
    
    def add_mesh_autoscaled(self, name, pose, filename):
        """
        An adding function that scales mm models to m automatically, by assuming
        that if any dimension is over 5 meters it should do so. 
        """
        scale = [1,1,1]
        collision_object = self._PlanningSceneInterface__make_mesh(name, pose, filename)
        for point in collision_object.meshes[0].vertices:
            if (abs(point.x) > 5 or abs(point.y) > 5 or abs(point.z) > 5):
                scale = [.001, .001, .001]
                break
        rospy.loginfo(self.__class__.__name__ + '::add_mesh_autoscaled::' +
                     'Sent mesh name: %s filename %s'%(name, filename))
        self.__scale_mesh(collision_object, scale)
        self._pub_co.publish(collision_object)


