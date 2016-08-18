using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using Microsoft.Kinect.Toolkit.FaceTracking;

namespace PVclr {
    static class Program {
        static HashSet<TcpClient> clients = new HashSet<TcpClient>();
        static Dictionary<string, int> kinectLookup = new Dictionary<string, int>();

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main() {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            new Thread(new ThreadStart(socketConnections)).Start();


            KinectSensor[] kinects = KinectSensor.KinectSensors.ToArray();
            for (int i = 0; i < kinects.Length; i++) {

                KinectSensor kinect = kinects[i];
                kinectLookup.Add(kinect.UniqueKinectId, i);
                kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                kinect.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
                try {
                    // This will throw on non Kinect For Windows devices.
                    kinect.DepthStream.Range = DepthRange.Near;
                    kinect.SkeletonStream.EnableTrackingInNearRange = true;
                }
                catch (InvalidOperationException) {
                    kinect.DepthStream.Range = DepthRange.Default;
                    kinect.SkeletonStream.EnableTrackingInNearRange = false;
                }

                kinect.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                kinect.SkeletonStream.Enable();
                KinectFrameHandler fh = new KinectFrameHandler();
                kinect.AllFramesReady += fh.kinect_AllFramesReady;
                kinect.Start();

            }

            Application.Run();
        }

        class KinectFrameHandler {
            private readonly Dictionary<int, SkeletonFaceTracker> trackedSkeletons = new Dictionary<int, SkeletonFaceTracker>();

            private byte[] colorImage;

            private ColorImageFormat colorImageFormat = ColorImageFormat.Undefined;

            private short[] depthImage;

            private DepthImageFormat depthImageFormat = DepthImageFormat.Undefined;

            private bool disposed;

            private Skeleton[] skeletonData;
            private const uint MaxMissedFrames = 100;

            public void kinect_AllFramesReady(object sender, AllFramesReadyEventArgs e) {

                KinectSensor kinect = (KinectSensor)sender;
                //Console.WriteLine(kinect.UniqueKinectId);
                ColorImageFrame colorImageFrame = null;
                DepthImageFrame depthImageFrame = null;
                SkeletonFrame skeletonFrame = null;

                try {
                    colorImageFrame = e.OpenColorImageFrame();
                    depthImageFrame = e.OpenDepthImageFrame();
                    skeletonFrame = e.OpenSkeletonFrame();

                    if (colorImageFrame == null || depthImageFrame == null || skeletonFrame == null) {
                        return;
                    }

                    // Check for image format changes.  The FaceTracker doesn't
                    // deal with that so we need to reset.
                    if (depthImageFormat != depthImageFrame.Format) {
                        ResetFaceTracking();
                        depthImage = null;
                        depthImageFormat = depthImageFrame.Format;
                    }

                    if (colorImageFormat != colorImageFrame.Format) {
                        ResetFaceTracking();
                        colorImage = null;
                        colorImageFormat = colorImageFrame.Format;
                    }

                    // Create any buffers to store copies of the data we work with
                    if (depthImage == null) {
                        depthImage = new short[depthImageFrame.PixelDataLength];
                    }

                    if (colorImage == null) {
                        colorImage = new byte[colorImageFrame.PixelDataLength];
                    }

                    // Get the skeleton information
                    if (skeletonData == null || skeletonData.Length != skeletonFrame.SkeletonArrayLength) {
                        skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    }

                    colorImageFrame.CopyPixelDataTo(colorImage);
                    depthImageFrame.CopyPixelDataTo(depthImage);
                    skeletonFrame.CopySkeletonDataTo(skeletonData);

                    // Update the list of trackers and the trackers with the current frame information
                    foreach (Skeleton skeleton in skeletonData) {
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked
                            || skeleton.TrackingState == SkeletonTrackingState.PositionOnly) {
                            // We want keep a record of any skeleton, tracked or untracked.
                            if (!this.trackedSkeletons.ContainsKey(skeleton.TrackingId)) {
                                this.trackedSkeletons.Add(skeleton.TrackingId, new SkeletonFaceTracker());
                            }

                            // Give each tracker the upated frame.
                            SkeletonFaceTracker skeletonFaceTracker;
                            bool tracked = false;
                            if (this.trackedSkeletons.TryGetValue(skeleton.TrackingId, out skeletonFaceTracker)) {

                                if (!tracked) {
                                    tracked |= skeletonFaceTracker.OnFrameReady(kinect, colorImageFormat, colorImage, depthImageFormat, depthImage, skeleton);
                                }
                                skeletonFaceTracker.LastTrackedFrame = skeletonFrame.FrameNumber;
                            }
                        }
                    }

                    RemoveOldTrackers(skeletonFrame.FrameNumber);

                }



                finally {
                    if (colorImageFrame != null) {
                        colorImageFrame.Dispose();
                    }

                    if (depthImageFrame != null) {
                        depthImageFrame.Dispose();
                    }

                    if (skeletonFrame != null) {
                        skeletonFrame.Dispose();
                    }
                }
            }

            private void RemoveTracker(int trackingId) {
                this.trackedSkeletons[trackingId].Dispose();
                this.trackedSkeletons.Remove(trackingId);
            }

            private void ResetFaceTracking() {
                foreach (int trackingId in new List<int>(this.trackedSkeletons.Keys)) {
                    this.RemoveTracker(trackingId);
                }
            }
            private void RemoveOldTrackers(int currentFrameNumber) {
                var trackersToRemove = new List<int>();

                foreach (var tracker in this.trackedSkeletons) {
                    uint missedFrames = (uint)currentFrameNumber - (uint)tracker.Value.LastTrackedFrame;
                    if (missedFrames > MaxMissedFrames) {
                        // There have been too many frames since we last saw this skeleton
                        trackersToRemove.Add(tracker.Key);
                    }
                }

                foreach (int trackingId in trackersToRemove) {
                    this.RemoveTracker(trackingId);
                }
            }
        }

        private class SkeletonFaceTracker : IDisposable {
            private static FaceTriangle[] faceTriangles;

            private EnumIndexableCollection<FeaturePoint, PointF> facePoints;

            private FaceTracker faceTracker;

            private bool lastFaceTrackSucceeded;

            private SkeletonTrackingState skeletonTrackingState;

            public int LastTrackedFrame { get; set; }

            public void Dispose() {
                if (this.faceTracker != null) {
                    this.faceTracker.Dispose();
                    this.faceTracker = null;
                }
            }


            /// <summary>
            /// Updates the face tracking information for this skeleton
            /// </summary>
            internal bool OnFrameReady(KinectSensor kinectSensor, ColorImageFormat colorImageFormat, byte[] colorImage, DepthImageFormat depthImageFormat, short[] depthImage, Skeleton skeletonOfInterest) {
                this.skeletonTrackingState = skeletonOfInterest.TrackingState;

                if (this.skeletonTrackingState != SkeletonTrackingState.Tracked) {
                    // nothing to do with an untracked skeleton.
                    return false;
                }

                if (this.faceTracker == null) {
                    try {
                        this.faceTracker = new FaceTracker(kinectSensor);
                    }
                    catch (InvalidOperationException) {
                        // During some shutdown scenarios the FaceTracker
                        // is unable to be instantiated.  Catch that exception
                        // and don't track a face.
                        Console.WriteLine("AllFramesReady - creating a new FaceTracker threw an InvalidOperationException");
                        this.faceTracker = null;
                    }
                }

                if (this.faceTracker != null) {
                    FaceTrackFrame frame = this.faceTracker.Track(
                        colorImageFormat, colorImage, depthImageFormat, depthImage, skeletonOfInterest);

                    this.lastFaceTrackSucceeded = frame.TrackSuccessful;
                    if (this.lastFaceTrackSucceeded) {
                        Console.WriteLine("FaceTracked!");
                        Vector3DF rotation = frame.Rotation;
                        Vector3DF translation = frame.Translation;
                        int kinectIndex = kinectLookup[kinectSensor.UniqueKinectId];

                        byte[] msg = System.Text.Encoding.ASCII.GetBytes("Kinect " + kinectIndex + " " + rotation.X + " " + rotation.Y + " " + rotation.Z + " " + translation.X + " " + translation.Y + " " + translation.Z + "\n");

                        HashSet<TcpClient> disposalSet = new HashSet<TcpClient>();
                        lock (clients) {
                            foreach (TcpClient client in clients) {
                                try {
                                    if (client.Connected) {
                                        NetworkStream stream = client.GetStream();
                                        stream.Write(msg, 0, msg.Length);
                                        Console.WriteLine(System.Text.Encoding.ASCII.GetString(msg));
                                        stream.Flush();
                                        
                                    }
                                    else {
                                        disposalSet.Add(client);
                                    }
                                }
                                catch (Exception e) {
                                    Console.WriteLine(e);
                                }
                            }
                            foreach (TcpClient client in disposalSet) {
                                clients.Remove(client);
                            }
                        }
                        return true;
                    }
                }
                return false;
            }

            private struct FaceModelTriangle {
                public Point P1;
                public Point P2;
                public Point P3;
            }
        }


        static void socketConnections() {
            int port = 61420;
            IPAddress localAddr = IPAddress.Parse("127.0.0.1");
            TcpListener tl = new TcpListener(port);
            tl.Start();
            while (true) {
                try {
                    TcpClient client = tl.AcceptTcpClient();
                    lock (clients) {
                        clients.Add(client);
                    }
                }
                catch (Exception e) {
                    Console.WriteLine(e);
                }
            }
        }
    }



}
