import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer as mj_view
import threading
import time
from .scene_monitor import SceneMonitor
from .image_publisher import MujocoCameraBridge
from rclpy.executors import MultiThreadedExecutor

class MujocoROSBridge(Node):
    def __init__(self, robot_info, camera_info, robot_controller):
        super().__init__('mujoco_ros_bridge')

        # robot_info = [xml, urdf, hz]
        self.xml_path = robot_info[0]
        self.urdf_path = robot_info[1]
        self.ctrl_freq = robot_info[2]

        # camera_info = [name, width, height, fps]
        self.camera_name = camera_info[0]
        self.width = camera_info[1]
        self.height = camera_info[2]
        self.fps = camera_info[3]
          
        self.rc = robot_controller

        # Mujoco 모델 로드
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.dt = 1 / self.ctrl_freq
        self.model.opt.timestep = self.dt
       
        self.sm = SceneMonitor(self.model, self.data)
        #self.hand_eye = MujocoCameraBridge(self.model, camera_info)
      
        self.ctrl_dof = 8 # 7 + 1
        self.ctrl_step = 0

        self.running = True
        self.lock = threading.Lock()
        self.robot_thread = threading.Thread(target=self.robot_control, daemon=True)
        self.hand_eye_thread = threading.Thread(target=self.hand_eye_control, daemon=True)
        self.ros_thread = threading.Thread(target=self.ros_control, daemon=True)


    # visualize thread = main thread
    def run(self):        
        scene_update_freq = 30
        try:     
            with mj_view.launch_passive(self.model, self.data) as viewer:            
                # self.sm.getAllObject()        
                # self.sm.getTargetObject()       
                # self.sm.getSensor() 
                self.robot_thread.start()    
                #self.hand_eye_thread.start()
                self.ros_thread.start()


                while self.running and viewer.is_running():   
                    start_time = time.perf_counter()       

                    with self.lock:                        
                        viewer.sync()  # 화면 업데이트          

                    self.time_sync(1/scene_update_freq, start_time, False)
                   
        except KeyboardInterrupt:
            print("\nSimulation interrupted. Closing viewer...")
            self.running = False
            self.robot_thread.join()
            #self.hand_eye_thread.join()
            self.ros_thread.join()
            self.sm.destroy_node()

    def robot_control(self):
        self.ctrl_step = 0

        try:
            while rclpy.ok() and self.running:            
                with self.lock:
                    start_time = time.perf_counter()                        

                    self.rc.updateModel(self.data, self.ctrl_step)                    
                    self.data.ctrl[:self.ctrl_dof] = self.rc.compute()   
                    mujoco.mj_step(self.model, self.data)  # 시뮬레이션 실행
                    
                    self.ctrl_step += 1
                    
                self.time_sync(self.dt, start_time, False)
            
        except KeyboardInterrupt:
            self.get_logger().into("\nSimulation interrupted. Closing robot controller ...")
            self.rc.destroy_node()

    def hand_eye_control(self):
        renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        hand_eye_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)

        while rclpy.ok() and self.running:            
            with self.lock:
                start_time = time.perf_counter()  
                renderer.update_scene(self.data, camera=hand_eye_id)
                #self.hand_eye.getImage(renderer.render(), self.ctrl_step)     

            self.time_sync(1/self.fps, start_time, False)
        #self.hand_eye.destroy_node()

    def time_sync(self, target_dt, t_0, verbose=False):
        elapsed_time = time.perf_counter() - t_0
        sleep_time = target_dt - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        if verbose:
            print(f'Time {elapsed_time*1000:.4f} + {sleep_time*1000:.4f} = {(elapsed_time + sleep_time)*1000} ms')
    
    def ros_control(self):
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(self.rc.tm)
        executor.add_node(self.rc.jm)
        #executor.add_node(self.hand_eye)
        executor.spin()
        executor.shutdown()

        self.rc.tm.destroy_node()
        self.rc.jm.destroy_node()