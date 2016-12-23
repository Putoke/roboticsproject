using System;
using DobotClientDemo.CPlusDll;

namespace Jackbot {
    class Program {

        private static Pose pose = new Pose();
        private static bool isConnected = false;

        [STAThread]
        static void Main(string[] args) {

            PTPCmd pdbCmd;
            UInt64 cmdIndex = 0;
            pdbCmd.ptpMode = (byte) PTPMode.PTPMOVLXYZMode;
            pdbCmd.x = 100;
            pdbCmd.y = -200;
            pdbCmd.z = -53;
            pdbCmd.rHead = 0;

            StartDobot();

            DobotDll.GetPose(ref pose);
            
            //DobotDll.SetPTPCmd(ref pdbCmd, false, ref cmdIndex);

            Console.WriteLine("pose params: ");
            System.Threading.Thread.Sleep(1000);

        }

        private static void StartDobot() {
            int ret = DobotDll.ConnectDobot("", 115200);
            // start connect
            if (ret != (int)DobotConnect.DobotConnect_NoError)
            {
                Console.WriteLine("Connect error");
                return;
            }
            Console.WriteLine("Connect success");

            isConnected = true;
            DobotDll.SetCmdTimeout(3000);

            // Must set when sensor is not exist
            DobotDll.ResetPose(true, 45, 45);


            UInt64 cmdIndex = 0;
            JOGJointParams jsParam;
            jsParam.velocity = new float[] { 200, 200, 200, 200 };
            jsParam.acceleration = new float[] { 200, 200, 200, 200 };
            DobotDll.SetJOGJointParams(ref jsParam, false, ref cmdIndex);

            JOGCommonParams jdParam;
            jdParam.velocityRatio = 100;
            jdParam.accelerationRatio = 100;
            DobotDll.SetJOGCommonParams(ref jdParam, false, ref cmdIndex);

            PTPJointParams pbsParam;
            pbsParam.velocity = new float[] { 200, 200, 200, 200 };
            pbsParam.acceleration = new float[] { 200, 200, 200, 200 };
            DobotDll.SetPTPJointParams(ref pbsParam, false, ref cmdIndex);

            PTPCoordinateParams cpbsParam;
            cpbsParam.xyzVelocity = 100;
            cpbsParam.xyzAcceleration = 100;
            cpbsParam.rVelocity = 100;
            cpbsParam.rAcceleration = 100;
            DobotDll.SetPTPCoordinateParams(ref cpbsParam, false, ref cmdIndex);

            PTPJumpParams pjp;
            pjp.jumpHeight = 20;
            pjp.zLimit = 100;
            DobotDll.SetPTPJumpParams(ref pjp, false, ref cmdIndex);

            PTPCommonParams pbdParam;
            pbdParam.velocityRatio = 30;
            pbdParam.accelerationRatio = 30;
            DobotDll.SetPTPCommonParams(ref pbdParam, false, ref cmdIndex);
        }
    }
}
