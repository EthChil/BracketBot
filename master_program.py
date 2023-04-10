from multiprocessing import Process, Manager

from odrive_setup_process import setup_odrive
from odrive_brake_process import brake_both_motors
from imu_setup_process import setup_imu
from ego_motion import EGO
from balance_process import LQR


if __name__ == "__main__":
    try:
        with Manager() as manager:
            
            odrive_setup_process = Process(target=setup_odrive, args=())
            odrive_setup_process.start()
            odrive_setup_process.join()
            
            imu_setup_process = Process(target=setup_imu, args=())
            imu_setup_process.start()
            imu_setup_process.join()
            
            position_list = manager.list([None])
            ego_process = Process(target=EGO, args=(position_list,))
            ego_process.start()
            ego_process.join()
            
            balance_process = Process(target=LQR, args=())
            balance_process.start()
            balance_process.join()

    except KeyboardInterrupt:
        print("Brake Process Starting")
        brake_process = Process(target=brake_both_motors, args=())
        brake_process.start()
        brake_process.join()
        print("Brake Process Done")
        
        
        print("Stopping the EGO process...")
        ego_process.terminate()
        ego_process.join()
        print("EGO process stopped.")
        
    
