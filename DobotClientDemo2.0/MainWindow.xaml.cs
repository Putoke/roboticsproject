using System;
using System.Windows;
using System.Windows.Threading;
using DobotClientDemo.CPlusDll;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.UI;
using Emgu.CV.Util;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.IO;
using System.Drawing;
using System.Collections.Generic;
using System.Windows.Forms;

namespace DobotClientDemo
{
    public partial class MainWindow
    {
        private bool isConnectted = false;
        private Pose pose = new Pose();
        private System.Timers.Timer posTimer = new System.Timers.Timer();
        private WriteableBitmap colorBitmap;
        private ImageViewer viewer;
        private List<Card> library = new List<Card>();
        private MultiSourceFrameReader reader;
        
        public MainWindow()
        {
            this.Hide();
            StartPeriodic();
            StartDobot();
            viewer = new ImageViewer();
            viewer.SetBounds(0, 0, 1920, 1080);

            KinectSensor sensor = KinectSensor.GetDefault();
            ColorFrameReader frameReader = sensor.ColorFrameSource.OpenReader();
            frameReader.FrameArrived += Reader_ColorFrameArrived;
            reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color |
                                             FrameSourceTypes.Depth |
                                             FrameSourceTypes.Infrared |
                                             FrameSourceTypes.Body);
            FrameDescription colorFrameDescription = sensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            sensor.Open();

            dobotStuff();

            Bitmap img = BitmapFromWriteableBitmap(colorBitmap);
            Image<Bgr, Byte> myImage = new Image<Bgr, Byte>(img);

            loadLibrary(CvInvoke.Imread("hearts (2).png"), 1);
            loadLibrary(CvInvoke.Imread("diamonds (2).png"), 2);
            loadLibrary(CvInvoke.Imread("clubs (2).png"), 3);
            loadLibrary(CvInvoke.Imread("spades (2).png"), 4);

            //Mat image = CvInvoke.Imread("testbild-cropped.png");
            //detectCards(myImage.Mat, 1);

            viewer.ShowDialog();

            Environment.Exit(0);
        }

        private void dobotStuff()
        {
            PTPJointParams ptpParams;
            ptpParams.acceleration = new float[4] { 200, 200, 200, 200 };
            ptpParams.velocity = new float[4] { 200, 200, 200, 200 };

            PTPCoordinateParams ptpCParams;
            ptpCParams.rAcceleration = 200;
            ptpCParams.rVelocity = 200;
            ptpCParams.xyzAcceleration = 200;
            ptpCParams.xyzVelocity = 200;

            PTPJumpParams ptpJParams;
            ptpJParams.jumpHeight = 10;
            ptpJParams.zLimit = 200;

            PTPCommonParams ptpCmParams;
            ptpCmParams.accelerationRatio = 100;
            ptpCmParams.velocityRatio = 100;

            UInt64 cmdIndex = 0;
            WAITCmd wait;
            wait.timeout = 2000;

            HOMECmd hmcmd = new HOMECmd();
            DobotDll.SetHOMECmd(ref hmcmd, false, ref cmdIndex);
            System.Threading.Thread.Sleep(20000);
            Pose pose = new Pose();
            DobotDll.GetPose(ref pose);
            Console.WriteLine(pose.x + ", " + pose.y + ", " + pose.z);
            DobotDll.SetPTPJointParams(ref ptpParams, false, ref cmdIndex);
            DobotDll.SetPTPCoordinateParams(ref ptpCParams, false, ref cmdIndex);
            DobotDll.SetPTPJumpParams(ref ptpJParams, false, ref cmdIndex);
            DobotDll.SetPTPCommonParams(ref ptpCmParams, false, ref cmdIndex);

            pickNewCard(cmdIndex, wait);

            placeCard(290, 0, cmdIndex);

            pickNewCard(cmdIndex, wait);

            placeCard(200, 0, cmdIndex);

            pickNewCard(cmdIndex, wait);

            placeCard(290, -50, cmdIndex);

            pickNewCard(cmdIndex, wait);

            placeCard(290, -100, cmdIndex);
        }

        private void placeCard(int x, int y, UInt64 cmdIndex)
        {
            moveDobot(x, y, -65, cmdIndex);
            DobotDll.SetEndEffectorSuctionCup(true, false, true, ref cmdIndex);
            moveDobot(x, y, -10, cmdIndex);
        }

        private void pickNewCard(UInt64 cmdIndex, WAITCmd wait)
        {
            moveDobot(220, 140, -10, cmdIndex);
            moveDobot(220, 140, -50, cmdIndex);
            DobotDll.SetEndEffectorSuctionCup(true, true, true, ref cmdIndex);
            moveDobot(220, 140, -10, cmdIndex);
            DobotDll.SetWAITCmd(ref wait, true, ref cmdIndex);
        }

        private void moveDobot(int x, int y, int z, UInt64 cmdIndex)
        {
            PTPCmd ptpCmd;
            ptpCmd.ptpMode = (byte)PTPMode.PTPMOVLXYZMode;
            ptpCmd.x = x;
            ptpCmd.y = y;
            ptpCmd.z = z;
            ptpCmd.rHead = 0;
            DobotDll.SetPTPCmd(ref ptpCmd, true, ref cmdIndex);   
        }

        private void detectCards(Mat image, int cardsToDetect)
        {
            Mat gray = new Mat(), blur = new Mat(), thresh = new Mat();
            CvInvoke.CvtColor(image, gray, ColorConversion.Bgr2Gray);
            CvInvoke.GaussianBlur(gray, blur, new System.Drawing.Size(1, 1), 1000);
            CvInvoke.Threshold(blur, thresh, 200, 255, ThresholdType.Binary);
            Mat hierarchy = new Mat();
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(thresh, contours, hierarchy, RetrType.Tree, ChainApproxMethod.ChainApproxNone);
            for (int i = 0; i < contours.Size; ++i)
            {
                int j = i;
                while (j > 0 && CvInvoke.ContourArea(contours[j - 1], false) < CvInvoke.ContourArea(contours[j], false))
                {
                    VectorOfPoint tmp = new VectorOfPoint();
                    tmp.Push(contours[j]);
                    contours[j].Clear();
                    contours[j].Push(contours[j - 1]);
                    contours[j - 1].Clear();
                    contours[j - 1].Push(tmp);
                    j--;
                }
            }

            List<RotatedRect> foundCards = new List<RotatedRect>();

            Image<Bgr, byte> tst = new Image<Bgr, byte>(image.Bitmap);

            int numCards = 0;
            for (int i = 0; i < contours.Size && numCards < cardsToDetect; ++i)
            {
                VectorOfPoint card = contours[i];
                double area = CvInvoke.ContourArea(card, false);

                double peri = CvInvoke.ArcLength(card, true);
                VectorOfPoint approx = new VectorOfPoint();
                CvInvoke.ApproxPolyDP(card, approx, 0.02 * peri, true);
                RotatedRect rect = CvInvoke.MinAreaRect(card);
                System.Drawing.PointF[] r = CvInvoke.BoxPoints(rect);

                bool stop = false;
                for (int j = 0; j < foundCards.Count; ++j)
                {
                    RotatedRect crd = foundCards[j];
                    System.Drawing.PointF center = crd.Center;
                    if (rect.MinAreaRect().Left < center.X && rect.MinAreaRect().Right > center.X && rect.MinAreaRect().Top < center.Y && rect.MinAreaRect().Bottom > center.Y)
                    {
                        stop = true;
                    }

                }
                if (stop)
                    continue;

                numCards++;
                System.Drawing.PointF[] points = new System.Drawing.PointF[4], points2 = new System.Drawing.PointF[4];
                points[0] = new System.Drawing.PointF(0, 0);
                points[1] = new System.Drawing.PointF(799, 0);
                points[2] = new System.Drawing.PointF(799, 799);
                points[3] = new System.Drawing.PointF(0, 799);

                for (int j = 0; j < approx.Size && j < 4; ++j)
                {
                    points2[j] = approx[j];
                }

                Array.Sort(points2, (a, b) => (int)(a.Y - b.Y));
                if (points2[0].X < points2[1].X)
                {
                    System.Drawing.PointF tmp = points2[0];
                    points2[0] = points2[1];
                    points2[1] = tmp;
                }
                if (points2[2].X > points2[3].X)
                {
                    System.Drawing.PointF tmp = points2[2];
                    points2[2] = points2[3];
                    points2[3] = tmp;
                }

                Mat transform = CvInvoke.GetPerspectiveTransform(points2, points);
                Mat warp = new Mat();
                CvInvoke.WarpPerspective(blur, warp, transform, new System.Drawing.Size(800, 800));
                ImageViewer viewer = new ImageViewer(warp);
                viewer.ShowDialog();
                Card recognizedCard = null;
                foreach(var c in library)
                {
                    if (recognizedCard == null)
                        recognizedCard = library[16];
                    Mat diff1 = new Mat(), diff2 = new Mat();
                    CvInvoke.AbsDiff(recognizedCard.GetImage(), warp, diff1);
                    CvInvoke.AbsDiff(c.GetImage(), warp, diff2);
                    var data1 = diff1.GetData();
                    int sum1 = 0;
                    foreach(var d in data1)
                    {
                        sum1 += d;
                    }
                    var data2 = diff2.GetData();
                    int sum2 = 0;
                    foreach (var d in data2)
                    {
                        sum2 += d;
                    }
                    if(sum2 <= sum1)
                    {
                        recognizedCard = c;
                    }
                }
                Console.WriteLine(recognizedCard.GetColor() + ", " + recognizedCard.GetValue());

                foundCards.Add(rect);
            }

        }

        private void loadLibrary(Mat image, int color)
        {
            Mat gray = new Mat(), blur = new Mat(), thresh = new Mat();
            CvInvoke.CvtColor(image, gray, ColorConversion.Bgr2Gray);
            CvInvoke.GaussianBlur(gray, blur, new System.Drawing.Size(1, 1), 1000);
            CvInvoke.Threshold(blur, thresh, 200, 255, ThresholdType.Binary);
            Mat hierarchy = new Mat();
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(thresh, contours, hierarchy, RetrType.Tree, ChainApproxMethod.ChainApproxNone);

            List<RotatedRect> foundCards = new List<RotatedRect>();

            for (int i = 0; i < contours.Size; ++i)
            {
                int j = i;
                while (j > 0 && CvInvoke.ContourArea(contours[j - 1], false) < CvInvoke.ContourArea(contours[j], false))
                {
                    VectorOfPoint tmp = new VectorOfPoint();
                    tmp.Push(contours[j]);
                    contours[j].Clear();
                    contours[j].Push(contours[j - 1]);
                    contours[j - 1].Clear();
                    contours[j - 1].Push(tmp);
                    j--;
                }
            }

            Image<Bgr, byte> tst = new Image<Bgr, byte>(image.Bitmap);

            int numCards = 0;
            for (int i = 0; i < contours.Size && numCards < 13; ++i)
            {
                VectorOfPoint card = contours[i];
                double area = CvInvoke.ContourArea(card, false);

                double peri = CvInvoke.ArcLength(card, true);
                VectorOfPoint approx = new VectorOfPoint();
                CvInvoke.ApproxPolyDP(card, approx, 0.02 * peri, true);
                RotatedRect rect = CvInvoke.MinAreaRect(card);
                System.Drawing.PointF[] r = CvInvoke.BoxPoints(rect);

                bool stop = false;
                for (int j = 0; j < foundCards.Count; ++j)
                {
                    RotatedRect crd = foundCards[j];
                    System.Drawing.PointF center = crd.Center;
                    if (rect.MinAreaRect().Left < center.X && rect.MinAreaRect().Right > center.X && rect.MinAreaRect().Top < center.Y && rect.MinAreaRect().Bottom > center.Y)
                    {
                        stop = true;
                    }

                }
                if (stop)
                    continue;

                numCards++;
                System.Drawing.PointF[] points = new System.Drawing.PointF[4], points2 = new System.Drawing.PointF[4];
                points[0] = new System.Drawing.PointF(0, 0);
                points[1] = new System.Drawing.PointF(799, 0);
                points[2] = new System.Drawing.PointF(799, 799);
                points[3] = new System.Drawing.PointF(0, 799);

                for (int j = 0; j < approx.Size && j < 4; ++j)
                {
                    points2[j] = approx[j];
                }

                Array.Sort(points2, (a, b) => (int)(a.Y - b.Y));
                if(points2[0].X < points2[1].X)
                {
                    System.Drawing.PointF tmp = points2[0];
                    points2[0] = points2[1];
                    points2[1] = tmp;
                }
                if (points2[2].X > points2[3].X)
                {
                    System.Drawing.PointF tmp = points2[2];
                    points2[2] = points2[3];
                    points2[3] = tmp;
                }

                Mat transform = CvInvoke.GetPerspectiveTransform(points2, points);
                Mat warp = new Mat();
                CvInvoke.WarpPerspective(blur, warp, transform, new System.Drawing.Size(800, 800));

                //ImageViewer asd = new ImageViewer(warp);
                //asd.ShowDialog();
                switch(i)
                {
                    case 0:
                        library.Add(new Card(warp, color, 10));
                        break;
                    case 1:
                        library.Add(new Card(warp, color, 11));
                        break;
                    case 2:
                        library.Add(new Card(warp, color, 12));
                        break;
                    case 3:
                        library.Add(new Card(warp, color, 13));
                        break;
                    case 4:
                        library.Add(new Card(warp, color, 5));
                        break;
                    case 5:
                        library.Add(new Card(warp, color, 6));
                        break;
                    case 6:
                        library.Add(new Card(warp, color, 7));
                        break;
                    case 7:
                        library.Add(new Card(warp, color, 9));
                        break;
                    case 8:
                        library.Add(new Card(warp, color, 8));
                        break;
                    case 9:
                        library.Add(new Card(warp, color, 14));
                        break;
                    case 10:
                        library.Add(new Card(warp, color, 2));
                        break;
                    case 11:
                        library.Add(new Card(warp, color, 3));
                        break;
                    case 12:
                        library.Add(new Card(warp, color, 4));
                        break;
                }
                
                foundCards.Add(rect);
            }
        }

        private System.Drawing.Bitmap BitmapFromWriteableBitmap(WriteableBitmap writeBmp)
        {
            System.Drawing.Bitmap bmp;
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create((BitmapSource)writeBmp));
                enc.Save(outStream);
                bmp = new System.Drawing.Bitmap(outStream);
            }
            return bmp;
        }

        private void Reader_MultiSourceFramearrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    var bodies = new Body[frame.BodyFrameSource.BodyCount];
                    frame.GetAndRefreshBodyData(bodies);

                    foreach(var body in bodies)
                    {
                        if(body.IsTracked)
                        {
                            Joint rightHand = body.Joints[JointType.HandRight];
                            Joint leftHand = body.Joints[JointType.HandLeft];
                            // Find the right hand state
                            switch (body.HandRightState)
                            {
                                case HandState.Open:
                                    Console.WriteLine("open");
                                    break;
                                case HandState.Closed:
                                    Console.WriteLine("closed");
                                    break;
                                case HandState.Lasso:
                                    break;
                                case HandState.Unknown:
                                    break;
                                case HandState.NotTracked:
                                    break;
                                default:
                                    break;
                            }

                            // Find the left hand state
                            switch (body.HandLeftState)
                            {
                                case HandState.Open:
                                    break;
                                case HandState.Closed:
                                    break;
                                case HandState.Lasso:
                                    break;
                                case HandState.Unknown:
                                    break;
                                case HandState.NotTracked:
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
            }
        }

        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();

                        //Bitmap img = BitmapFromWriteableBitmap(colorBitmap);
                        //Image<Bgr, Byte> myImage = new Image<Bgr, Byte>(img);
                        //var cropped = myImage.GetSubRect(new Rectangle(800, 800, 400, 280)).Rotate(90, new Bgr());
                        //detectCards(cropped.Mat, 1);
                        //viewer.Image = cropped;
                        //viewer.Update();
                    }
                }
            }
        }

        /// <summary>
        /// StartDobot
        /// </summary>
        private void StartDobot()
        {
            int ret = DobotDll.ConnectDobot("", 115200);
            // start connect
            if (ret != (int)DobotConnect.DobotConnect_NoError)
            {
                Console.WriteLine("Connect error", MsgInfoType.Error);
                return;
            }
            Console.WriteLine("Connect success", MsgInfoType.Info);

            isConnectted = true;
            DobotDll.SetCmdTimeout(3000);

            // Must set when sensor is not exist
            DobotDll.ResetPose(true, 45, 45);


            UInt64 cmdIndex = 0;
            JOGJointParams jsParam;
            jsParam.velocity = new float[]{200, 200, 200, 200};
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
            pjp.jumpHeight= 20;
            pjp.zLimit = 100;
            DobotDll.SetPTPJumpParams(ref pjp, false, ref cmdIndex);

            PTPCommonParams pbdParam;
            pbdParam.velocityRatio = 30;
            pbdParam.accelerationRatio = 30;
            DobotDll.SetPTPCommonParams(ref pbdParam, false, ref cmdIndex);
        }

        /// <summary>
        /// StartPeriodic
        /// </summary>
        private void StartPeriodic()
        {
            // start peridic cmd
            DispatcherTimer cmdTimer = new DispatcherTimer();
            cmdTimer.Interval = new TimeSpan(0, 0, 0, 0, 10);
            cmdTimer.Tick += CmdTimer_Tick;
            cmdTimer.Start();


            posTimer.Elapsed += new System.Timers.ElapsedEventHandler(PosTimer_Tick);
            posTimer.Interval = 600;
            posTimer.Start();
        }

        private void PosTimer_Tick(object sender, System.Timers.ElapsedEventArgs e)
        {
            if (!isConnectted)
                return;

            DobotDll.GetPose(ref pose);
        }

        private void CmdTimer_Tick(object sender, EventArgs e)
        {
            // called in 200ms
            DobotDll.PeriodicTask();
        }

    }
}
