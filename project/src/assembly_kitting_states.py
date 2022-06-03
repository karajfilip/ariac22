from json.tool import main
import rospy
import smach
from geometry_msgs.msg import Pose, PoseArray
from math import pi
from nist_assembly import AssemblyPart
from nist_gear.msg import LogicalCameraImage

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
        gripper = self.act.gripper_type
        if str(gripper) !=  'gripper_part':
            ud.gripper = 'gripper_tray'
            return 'changegripper'
        else:
            return 'next'

class SendGantry(smach.State):
    def __init__(self, gantryplanner, processmgmt, assembly, sensors, outcomes=['arrived'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys)
        self.gp = gantryplanner
        self.node = processmgmt
        self.ass = assembly
        self.sen = sensors

    def execute(self, ud):
        
        if (str(ud.task.station_id) == "as2"):
            while self.sen.bb1:
                rospy.logwarn("Human obstacle at as2")
                rospy.sleep(0.2)
        if(str(ud.task.station_id) == "as4"):
            while self.sen.bb2:
                rospy.logwarn("Human obstacle at as4")
                rospy.sleep(0.2)


        self.gp.move(ud.task.station_id)
        while self.gp.checking_position:
            pass
        return 'arrived'

class SubmitAssemblyShipment(smach.State):
    def __init__(self, processmgmt, outcomes=['success'], input_keys=['task']):
        smach.State.__init__(self, outcomes, input_keys=input_keys)
        self.node = processmgmt
    
    def execute(self, ud):
        self.node.submit_assembly_shipment(ud.task.station_id, ud.task.shipment_type)
        return 'success'

class FindPartOnTray(smach.State):
    def __init__(self, actuators, processmgmt, sensors, outcomes=['found', 'noFound'], input_keys=['part', 'kittingtask'], output_keys=['partcurrentposition']):
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

            return 'noFound' 

    
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
        #gantry_position = self.ass.get_gantry_position()
        gantry_orientation = "right"
        for i in range(1,5):
            if  self.node.get_position_AGV('agv'+str(i)) == ud.task.station_id:
                pose_tray = self.sen.tf_transform(str("kit_tray_"+str(i)))

        ### TODO: smisli bolju logiku gdje orijentirat gantry
        if (str(ud.task.station_id) == "as1"):
            if(self.node.get_position_AGV("agv1") == "as1"):
                gantry_orientation = "right"
            else:
                gantry_orientation = "left"
        if(str(ud.task.station_id) == "as2"):    
            if (self.node.get_position_AGV("agv1") == "as2"):
                gantry_orientation = "right"
            else:
                gantry_orientation = "left"
        if(str(ud.task.station_id) == "as3"):
            if(self.node.get_position_AGV("agv3") == "as3"):
                gantry_orientation = "right"
            else:
                gantry_orientation = "left"
        if(str(ud.task.station_id) == "as4"):
            if (self.node.get_position_AGV("agv3") == "as4"):
                gantry_orientation = "right"
            else:
                gantry_orientation = "left"

        parts = self.node.filipov_process_assembly_shipment(self.node.orders[0].assembly_shipments[0])
            
        elem=parts[counter]
        
        gantry_pose = []
        gantry_pose = self.act.gantry_torso_state

        #gantry_pose.desired.positions[1]

        # spusti torso po x osi
        self.rm.move_torso([gantry_pose.desired.positions[0]+0.55, gantry_pose.desired.positions[1], pi/2])
        rospy.sleep(3)

        # ovisno o tome koji je agv pomakni torso blize njemu
        gantry_pose=self.act.gantry_torso_state
        if (gantry_orientation=="right"):
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1], 0.0])
            rospy.sleep(3)
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1] - 0.9, 0.0])
            rospy.sleep(3)
        else:
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1], pi])
            rospy.sleep(3)
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1] + 0.9, pi])
            rospy.sleep(3)

        rospy.logerr("Dosao do agv")

        # set part position
        part_curr_pose = Pose()
        part_curr_pose.position.x = ud.partcurrentposition.x 
        part_curr_pose.position.y = ud.partcurrentposition.y 
        part_curr_pose.position.z = ud.partcurrentposition.z 

        # pickup part
        #self.rm.pickup_gantry([part_curr_pose.position.x, part_curr_pose.position.y, part_curr_pose.position.z, 0, pi/2, 0],1) #,1
        #while not self.rm.gantry_pickedup:
        #    pass

        # podigni ruku prije kretanja    
        #self.rm.move_directly_gantry([part_curr_pose.position.x, part_curr_pose.position.y, part_curr_pose.position.z + 0.2, 0, pi/2, 0])
        #rospy.sleep(3)
        
        ## Put nazad do briefcase-a
        gantry_pose = self.act.gantry_torso_state

        if (gantry_orientation == "right"):
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1] + 0.9, gantry_pose.desired.positions[2]])
            rospy.sleep(2)
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1] + 0.9, pi/2])
            rospy.sleep(2)
        else:
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1] - 0.9, gantry_pose.desired.positions[2]])
            rospy.sleep(2)
            self.rm.move_torso([gantry_pose.desired.positions[0], gantry_pose.desired.positions[1] - 0.9, pi/2])
            rospy.sleep(2)
        
        ## odi do assembly tocke
        self.gp.move(ud.task.station_id)
        gantry_pose = self.act.gantry_torso_state
        if("sensor" in ud.part.type):
            self.rm.move_torso([gantry_pose.desired.positions[0]-0.2, gantry_pose.desired.positions[1], pi/2])
        else:
            self.rm.move_torso([gantry_pose.desired.positions[0]-0.3, gantry_pose.desired.positions[1], pi/2])

        

        ## Postavljanje elemenata u briefcase
        part_pose = Pose()
        part_pose.position.x = ud.part.pose.position.x + pose_briefase.position.x
        part_pose.position.y = ud.part.pose.position.y + pose_briefase.position.y
        part_pose.position.z = ud.part.pose.position.z + pose_briefase.position.z
        
        self.ass.move_gantry_to_assembly_position(briefcase, elem)
        #self.ass.move_gantry(-10.85, 3.23, pi/2)
        rospy.logwarn("Moved to assembly position.")
        #print(parttopick)
        #print("jedan print izmedu")
        #print(briefcase)
        self.ass.assemble_part(briefcase, elem)
        rospy.loginfo("Part assembled.")

        self.ass.move_arm_to_home_position()
        #self.ass.move_gantry_to_station(ud.task.station_id)
        #self.ass.orient_gantry(-3.05)
        counter += 1


        

        #self.gp.move(ud.task.station_id)
        #while self.gp.checking_position:
        #    pass

        #self.rm.place_gantry([part_pose.position.x, part_pose.position.y, part_pose.position.z, 0, pi/2, 0])

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
        self.rm.move_directly_gantry([curr_pose[0], curr_pose[1]+0.2, curr_pose[2]+0.3, 0, pi/2, 0], 1)
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
    def __init__(self, sensors, outcomes=['found', 'none'], input_keys=['task', 'part'], output_keys=['partposition', 'partcurrentposition']):  
        smach.State.__init__(self, outcomes, input_keys, output_keys)   
        self.sen = sensors  

    def execute(self, ud):  
        objects = self.sen.get_object_pose_in_workcell()    
        pose_tray = self.sen.tf_transform(str("kit_tray_" + str((ud.task.agv)[-1])))    
        part_pose = Pose()  
        part_pose.position.x = ud.part.pose.position.x + pose_tray.position.x   
        part_pose.position.y = ud.part.pose.position.y + pose_tray.position.y   
        part_pose.position.z = ud.part.pose.position.z + pose_tray.position.z   
        ud.partposition = part_pose.position    
        for product in objects: 
            if product.pose.position.x < - 2.7: 
                print("ODBACUJEM OVAJ OBJEKT NA POZICIJI " + str(product.pose.position.x))  
                continue    
            if product.type == ud.part.type:    
                ud.partcurrentposition = product.pose.position  
                return 'found'  
        return 'none'   

class KittingRobotPickAndPlace(smach.State):    
    def __init__(self, robotmover, sensors, outcomes=['success'], input_keys=['task', 'partposition', 'partcurrentposition']):  
        smach.State.__init__(self, outcomes, input_keys)    
        self.rm = robotmover    
        self.sen = sensors  

    def execute(self, ud):  
        partcurrentpos = [ud.partcurrentposition.x, ud.partcurrentposition.y, ud.partcurrentposition.z, 0, pi/2, 0] 
        partpos = [ud.partposition.x, ud.partposition.y, ud.partposition.z + 0.02, 0, pi/2, 0]  
        self.rm.pickup_kitting(partcurrentpos)  
        while not self.rm.kitting_pickedup: 
            rospy.sleep(0.2)    
        self.rm.place_kitting(partpos)  
        partpos[2] = partpos[2] + 0.3   
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
