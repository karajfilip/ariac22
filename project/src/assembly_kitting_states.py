import rospy
import smach
from geometry_msgs.msg import Pose, PoseArray
from math import pi
from nist_assembly import AssemblyPart
from nist_gear.msg import LogicalCameraImage

# BRISI
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
# DO TUD


counter = 0

class CheckPart(smach.State):
    def __init__(self, outcomes=['noParts', 'newPart'], input_keys=['task'], output_keys=['part']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.i = 0

    def execute(self, ud):
        if self.i < len(ud.task.products):
            ud.part = ud.task.products[self.i]
            self.i += 1
            return 'newPart'
        else:
           return 'noParts'

## ASSEMBLY STATES
class CheckGripper(smach.State):
    def __init__(self, actuators, outcomes=['changegripper', 'next'], input_keys=['task'], output_keys=['gripper']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.act = actuators

    def execute(self, ud):
        print(ud.task.shipment_type)
        gripper = self.act.gripper_type
        if str(gripper) !=  'gripper_part':
            ud.gripper = 'gripper_tray'
            return 'changegripper'
        else:
            return 'next'

class SendGantry(smach.State):
    def __init__(self, gantryplanner, processmgmt, assembly, outcomes=['arrived'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.gp = gantryplanner
        self.node = processmgmt
        self.ass = assembly

    def execute(self, ud):
        self.gp.move(ud.task.station_id)
        while self.gp.checking_position:
            rospy.sleep(0.2)
        return 'arrived'

class SubmitAssemblyShipment(smach.State):
    def __init__(self, processmgmt, outcomes=['success'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = processmgmt
    
    def execute(self, ud):
        self.node.submit_assembly_shipment(ud.task.station_id)
        return 'success'

class FindPartOnTray(smach.State):
    def __init__(self, actuators, processmgmt, sensors, outcomes=['found'], input_keys=['part', 'kittingtask'], output_keys=['partcurrentposition']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.act = actuators
        self.node = processmgmt
        self.sen = sensors

    def execute(self, ud):
        if ud.kittingtask:
            for product in ud.kittingtask.products:
                if product.type == ud.part.type:
                    ud.partcurrentposition = product.pose.position
                    return 'found'
        else:
            ud.partcurrentposition = Pose().position
            self.objects = self.sen.get_object_pose_in_workcell()
            print(self.objects)
            for product in self.objects:
                if product.type == ud.part.type:
                    ud.partcurrentposition = product.pose.position
                    print(product.pose.position)
                    return 'found'

            # if self.node.get_position_AGV("agv1") == "as2":
            #     self.node.move_AGV("agv1", "as1")
            #     while not self.node.get_position_AGV("agv1") == "as1":
            #         rospy.sleep(0.2)

            #     # subsc
            #     self.cam8 = self.sen.get_object_pose_in_workcell(8)

            #     for part in self.cam8:
            #         if part.type == ud.part.type:
            #             ud.partcurrentposition.x = part.pose.position.x - 4.990274
            #             ud.partcurrentposition.y = part.pose.position.y
            #             ud.partcurrentposition.z = part.pose.position.z

            #     self.node.move_AGV("agv1", "as2")
            #     while not self.node.get_position_AGV("agv1") == "as2":
            #         rospy.sleep(0.2)
            # ## AGV2
            # if self.node.get_position_AGV("agv2") == "as2":
            #     self.node.move_AGV("agv2", "as1")
            #     while not self.node.get_position_AGV("agv2") == "as1":
            #         rospy.sleep(0.2)

            #     # subsc
            #     self.cam9 = self.sen.get_object_pose_in_workcell(9)

            #     for part in self.cam9:
            #         if part.type == ud.part.type:
            #             ud.partcurrentposition.x = part.pose.position.x - 4.990274
            #             ud.partcurrentposition.y = part.pose.position.y
            #             ud.partcurrentposition.z = part.pose.position.z

            #     self.node.move_AGV("agv2", "as2")
            #     while not self.node.get_position_AGV("agv2") == "as2":
            #         rospy.sleep(0.2)
            # ## AGV3
            # if self.node.get_position_AGV("agv3") == "as4":
            #     self.node.move_AGV("agv3", "as3")
            #     while not self.node.get_position_AGV("agv3") == "as3":
            #         rospy.sleep(0.2)

            #    # subsc
            #     self.cam10 = self.sen.get_object_pose_in_workcell(10)

            #     for part in self.cam10:
            #         if part.type == ud.part.type:
            #             ud.partcurrentposition.x = part.pose.position.x - 4.990274
            #             ud.partcurrentposition.y = part.pose.position.y
            #             ud.partcurrentposition.z = part.pose.position.z

            #     self.node.move_AGV("agv3", "as4")
            #     while not self.node.get_position_AGV("agv3") == "as4":
            #         rospy.sleep(0.2)

            # ## AGV4
            # if self.node.get_position_AGV("agv4") == "as4":
            #     self.node.move_AGV("agv4", "as3")
            #     while not self.node.get_position_AGV("agv4") == "as3":
            #         rospy.sleep(0.2)

            #     # subsc
            #     self.cam11 = self.sen.get_object_pose_in_workcell(11)

            #     for part in self.cam11:
            #         if part.type == ud.part.type:
            #             ud.partcurrentposition.x = part.pose.position.x - 4.990274
            #             ud.partcurrentposition.y = part.pose.position.y
            #             ud.partcurrentposition.z = part.pose.position.z

            #     self.node.move_AGV("agv4", "as4")
            #     while not self.node.get_position_AGV("agv4") == "as4":
            #         rospy.sleep(0.2)

            return 'found' 

class GantryMovePart(smach.State):
    def __init__(self, robotmover, sensors, processmgmt, assembly, gantryplanner, act, outcomes=['moved'], input_keys=['partcurrentposition', 'part', 'task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.rm = robotmover
        self.sen = sensors
        self.node = processmgmt
        self.ass = assembly
        self.gp = gantryplanner
        self.act = act
    
    def execute(self, ud):
        global counter


        #if 'sensor' not in ud.part.type:
        #    return 'moved'
        # TORSO NA [-8.26, 0, -3.78]
        # A ONDA NA [-9 , 1.57, -3.08]

        # Priblizi agvu
        move_to = JointTrajectory()
        move_to.header.stamp = rospy.Time.now()
        move_to.joint_names = ["small_long_joint", "torso_base_main_joint", "torso_rail_joint"]
        point = JointTrajectoryPoint()
        point.positions = [-8.26, pi, -2.5]
        point.time_from_start = rospy.Duration(1)

        point2 = JointTrajectoryPoint()
        point2.positions = [-8.26, pi, -2.1]
        point2.time_from_start = rospy.Duration(2)

        move_to.points.append(point)
        move_to.points.append(point2)
        self.gp.trajectory_publisher.publish(move_to)

        rospy.sleep(2.2)

        # koji part???
        parts = self.node.filipov_process_assembly_shipment(self.node.orders[0].assembly_shipments[0])
        elem=parts[counter]
        print(ud.partcurrentposition)

        # Pickupaj part!
        if 'pump' in ud.part.type:
            self.rm.pickup_gantry([ud.partcurrentposition.x , ud.partcurrentposition.y, ud.partcurrentposition.z, 0, pi/2, 0], joints=1, liftup=0, object_name=ud.part.type)
        else:
            self.rm.pickup_gantry([ud.partcurrentposition.x , ud.partcurrentposition.y, ud.partcurrentposition.z, 0, pi/2, 0], joints=1, liftup=0)
        while not self.rm.gantry_pickedup:
            rospy.sleep(0.1)

        robot_pos = self.rm.inverse_kin.direct_kinematics_gantry_arm()
        offset = []
        offset.append(-robot_pos[0] + ud.partcurrentposition.x)
        offset.append(-robot_pos[1] + ud.partcurrentposition.y)
        #print("OFFSET JE: " + str(offset))

        # Digni ruku!
        self.rm.move_directly_gantry([ud.partcurrentposition.x, ud.partcurrentposition.y, ud.partcurrentposition.z + 0.8, 0, pi/2, 0], 1)
        rospy.sleep(1.5)
        if 'regulator' in ud.part.type:
            self.rm.move_directly_gantry([ud.partcurrentposition.x + 0.2, ud.partcurrentposition.y - 0.5, ud.partcurrentposition.z + 0.75, 0, pi, pi/2], 1)
            rospy.sleep(1.5)
            self.rm.move_directly_gantry([ud.partcurrentposition.x + 0.2, ud.partcurrentposition.y - 0.5, ud.partcurrentposition.z + 0.75, -pi/2, pi, pi/2], 1)
        elif 'sensor' in ud.part.type:
            self.rm.move_directly_gantry([ud.partcurrentposition.x + 0.2, ud.partcurrentposition.y - 0.5, ud.partcurrentposition.z + 0.75, pi/2, 0, pi/2], 1)
            rospy.sleep(1.5)
            self.rm.move_directly_gantry([ud.partcurrentposition.x + 0.2, ud.partcurrentposition.y - 0.3, ud.partcurrentposition.z + 0.75, 0, 0, pi], 1)
        else:
            self.rm.move_directly_gantry([ud.partcurrentposition.x + 0.2, ud.partcurrentposition.y - 0.5, ud.partcurrentposition.z + 0.75, 0, pi/2, 0], 1)
        rospy.sleep(1.2)

        # Pomakni se do kofera
        move_to = JointTrajectory()
        move_to.header.stamp = rospy.Time.now()
        move_to.joint_names = ["small_long_joint", "torso_base_main_joint", "torso_rail_joint"]
        point = JointTrajectoryPoint()
        point.positions = [-8.26 , 1.57, -2.9]
        point.time_from_start = rospy.Duration(1.5)

        point2 = JointTrajectoryPoint()
        if 'sensor' in ud.part.type:
            point2.positions = [ -8.55, 1.57, -2.9]
        else:
            point2.positions = [-9 , 1.57, -2.9]
        point2.time_from_start = rospy.Duration(3)

        move_to.points.append(point)
        move_to.points.append(point2)
        self.gp.trajectory_publisher.publish(move_to)

        rospy.sleep(3.2)

        # Stavi u kofer i makni se
        if 'pump' in ud.part.type:
            self.rm.assemble_gantry('as2', ud.part.type, joints=1, offset=offset)
        else:
            self.rm.assemble_gantry('as2', ud.part.type, joints=1)
        rospy.sleep(1.0)
        #self.ass.move_arm_to_home_position()

        counter += 1
        return 'moved'

## KITTING STATES
class CheckAGV(smach.State):
    def __init__(self, processmgmt, outcomes=['agvatks', 'agvnotatks'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = processmgmt

    def execute(self, ud):
        if (str(self.node.get_position_AGV(ud.task.agv)))[:-1] != 'ks':
            return 'agvnotatks'
        else:
            return 'agvatks'

class SendAGV(smach.State):
    def __init__(self, processmgmt, outcomes=['agvatks'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.node = processmgmt

    def execute(self, ud):
        self.node.move_AGV(ud.task.agv, 'ks')
        return super().execute(ud)

class CheckMoveableTray(smach.State):
    def __init__(self, actuators, outcomes=['changegripper', 'changetray'], input_keys=['task'], output_keys=['gripper']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.act = actuators

    def execute(self, ud):
        gripper = self.act.gripper_type
        if str(gripper) !=  'gripper_tray':
            ud.gripper = 'gripper_tray'
            return 'changegripper'
        else:
            return 'changetray'

class GetGripper(smach.State):
    def __init__(self, gantryplanner, robotmover, outcomes=['gripperon'], input_keys=['gripper']):
        smach.State.__init__(self, outcomes, input_keys)
        self.rm = robotmover
        self.gp = gantryplanner
    
    def execute(self, ud):   ##################### poboljsati?     kopija iz main.py
        curr_pose = self.rm.get_pos_gantry()
        self.rm.move_directly_gantry([curr_pose[0], curr_pose[1] + 0.2, curr_pose[2]+0.3, 0, pi/2, 0], 1)
        rospy.sleep(3)  # TODO pozicija i while

        self.gp.move('gripperstation')
        print("gantry je iznad gripper stationa.")

        while self.gp.checking_position:
            rospy.sleep(0.1)

        print(self.rm.inverse_kin.gripper_type)
        try:
            self.rm.inverse_kin.change_gripper(
                str(ud.gripper))
            rospy.logerr(self.rm.inverse_kin.gripper_type)
        except rospy.ServiceException as exc:
            print(str(exc))

        while(self.rm.inverse_kin.gripper_type != ud.gripper):
            rospy.sleep(0.2)
        return 'gripperon'

class GantryGetTray(smach.State):
    def __init__(self, gantryplanner, robotmover, sensors, outcomes=['trayon'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.gp = gantryplanner
        self.rm = robotmover
        self.sen = sensors

    def execute(self, ud):
        self.gp.move('traystation')
        while self.gp.checking_position:
            rospy.sleep(0.2)
        self.objects = self.sen.get_object_pose_in_workcell()
        min_tray = None
        ymin = 1111
        for tray in self.objects:
            if tray.type == ud.task.movable_tray.movable_tray_type:
                if tray.pose.position.y < ymin:
                    ymin = tray.pose.position.y
                    min_tray = tray
        tray = min_tray
        self.rm.pickup_gantry([tray.pose.position.x, tray.pose.position.y, tray.pose.position.z + 0.021, 0, pi/2, pi/2], joints=1, tray_pickup = 1, liftup=0)
        while not self.rm.gantry_pickedup:
            rospy.sleep(0.2)

        if tray.pose.position.x < -6: 
            self.rm.move_directly_gantry([tray.pose.position.x + 0.6 , tray.pose.position.y - 0.2, tray.pose.position.z + 0.4, 0, pi/2, pi/2], 1)
        else:
            self.rm.move_directly_gantry([tray.pose.position.x - 0.25 , tray.pose.position.y - 0.2, tray.pose.position.z + 0.4, 0, pi/2, pi/2], 1)

        rospy.sleep(1.0)
        self.gp.move(ud.task.agv)
        while self.gp.checking_position:
            rospy.sleep(0.2)

        agv_pose = self.sen.tf_transform(str("kit_tray_"+str((ud.task.agv)[-1])))

        self.rm.place_gantry([agv_pose.position.x, agv_pose.position.y, agv_pose.position.z, 0, pi/2, 0], 1, 0)
        rospy.sleep(0.1)
        if ud.task.agv == "agv1" or ud.task.agv == "agv2":
            self.rm.move_directly_gantry([agv_pose.position.x, agv_pose.position.y + 0.25, agv_pose.position.z + 0.4, 0, pi/2, 0], 1)
        else:
            self.rm.move_directly_gantry([agv_pose.position.x, agv_pose.position.y - 0.25, agv_pose.position.z + 0.4, 0, pi/2, 0], 1)
        rospy.sleep(0.2)
        self.gp.move('home')
        return 'trayon'

class FindPartInEnvironment(smach.State):
    def __init__(self, sensors, outcomes=['found'], input_keys=['part'], output_keys=['partposition', 'partcurrentposition']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.sen = sensors

    def execute(self, ud):
        objects = self.sen.get_object_pose_in_workcell()
        ud.partposition = ud.part.pose.position
        for product in objects:
            if product.pose.position.x < - 2.7:
                print("ODBACUJEM OVAJ OBJEKT NA POZICIJI " + str(product.pose.position.x))
                continue

            if product.type == ud.part.type:
                ud.partcurrentposition = product.pose.position
                return 'found'

class KittingRobotPickAndPlace(smach.State):
    def __init__(self, robotmover, sensors, outcomes=['success'], input_keys=['task', 'partposition', 'partcurrentposition']):
        smach.State.__init__(self, outcomes, input_keys)
        self.rm = robotmover
        self.sen = sensors

    def execute(self, ud):
        partcurrentpos = [ud.partcurrentposition.x, ud.partcurrentposition.y, ud.partcurrentposition.z, 0, pi/2, 0]
        pose_tray = self.sen.tf_transform(str("kit_tray_" + str((ud.task.agv)[-1])))
        part_pose = Pose()
        part_pose.position.x = ud.partposition.x + pose_tray.position.x
        part_pose.position.y = ud.partposition.y + pose_tray.position.y
        part_pose.position.z = ud.partposition.z + pose_tray.position.z
        partpos = [part_pose.position.x, part_pose.position.y, part_pose.position.z + 0.02, 0, pi/2, 0]
        self.rm.pickup_kitting(partcurrentpos)
        while not self.rm.kitting_pickedup:
            rospy.sleep(0.2)
        self.rm.place_kitting(partpos)
        partpos[2] = partpos[2] + 0.1
        self.rm.move_directly_kitting(partpos)
        return 'success'

class CheckFaulty(smach.State):
    def __init__(self, outcomes=['faulty', 'notfaulty'], input_keys=['part', 'kittingtask']):
        smach.State.__init__(self, outcomes, input_keys)

    def execute(self, ud):
        faultyparts = rospy.wait_for_message('ariac/quality_control_sensor_'+ud.kittingtask.agv[-1], LogicalCameraImage)
        print(ud.part)
        print(faultyparts.models)
        # if part in faultyparts.models:
        #     if ud.part.pose == part.pose:
        if len(faultyparts.models)>0:
                return 'faulty'
        return 'notfaulty'

class SubmitKittingShipment(smach.State):
    def __init__(self, processmgmt, outcomes=['success'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = processmgmt
    
    def execute(self, ud):
        self.node.submit_kitting_shipment(ud.task.agv, ud.task.assembly_station, ud.task.shipment_type)
        return 'success'

class WaitConveyorBelt(smach.State):
    def __init__(self, outcomes=['ontrack', 'finish'], input_keys=['traydone'], output_keys=['trackindex']):
        smach.State.__init__(self, outcomes, input_keys=input_keys, output_keys=output_keys)
        self.sub  = rospy.Subscriber('ariac/pose_on_track', PoseArray, self.cb)
        self.poselen = 0
        self.trackindex = 0

    def execute(self, ud):
        for ud.trackindex in self.trackindex:
            if ud.traydone:
                return 'finish'
        self.trackindex += 1
        
        return 'ontrack'
    
    def cb(self, msg):
        self.poselen = len(msg.poses)
    
class PickFromConveyor(smach.State):
    def __init__(self, robotmover, actuators, outcomes=['next'], input_keys=['task'], output_keys=['trackindex']):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.rm = robotmover
        self.bin1 = [-1.9, 3.37, 1, 0, pi/2, 0]
        self.bin5 = [-1.9, -3.37, 1, 0, pi/2, 0]
        self.trackindex = 0

    def execute(self, ud):
        self.rm.pickup_from_track(self.trackindex)
        if (ud.task.agv == 'agv1' or ud.task.agv == 'agv2'):
            self.rm.place_kitting(self.bin1)
        else:
            self.rm.place_kitting(self.bin5)
        self.rm.place_kitting([-0.56, 0.205, 1.4, 0, pi/2, 0])
        self.trackindex += 1
        ud.trackindex = self.trackindex
        return 'next'

class WaitKitting(smach.State):
    def __init__(self, actuators, outcomes=['done'], input_keys=['trackindex']):
        smach.State.__init__(self, outcomes, input_keys=input_keys) 
        self.act = actuators
        self.trackindex = 0
    
    def execute(self, ud):
        #homepose = [-0.56, 0.205, 1.4]
        #if (ud.trackindex > 0):
        #    while self.act.direct_kinematics_kitting_arm != homepose:
        #        pass
        return 'done'
    


