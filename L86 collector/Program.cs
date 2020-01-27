using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NMEA_Parser;
using System.IO;
using System.IO.Ports;
using System.Device.Location;
using System.Collections.Concurrent;
using System.Threading;
using System.Xml;
using System.Xml.Linq;
using System.Runtime.InteropServices;
using System.Diagnostics;
using CustumLoggers;
using PpsCardDelivery;

namespace L86_collector
{
    class Program
    {
        const int BAUD_RATE = 115200;

        private static void logTime(string folder)
        {
            while (!terminationEventSignal.IsCancellationRequested)
            {
                try
                {
                    using (StreamWriter wr = new StreamWriter(folder + "heartBeat.log", true))
                    {
                        wr.WriteLine(DateTime.Now.ToString("yyyy/MM/dd HH:mm:ss"));
                    }
                }
                catch (Exception e)
                {
                }
                Thread.Sleep(1000);
            }
        }

        private const string SoftwareVersion = "V5.7";

        static bool running = false;
        enum FixQuality
        {
            invalid = 0,
            fixGPS,
            fixDGPS,
            fixPPS,
            Real_Time_Kinematic,
            Float_RTK,
            estimated_dead_reckoning,
            Manual_input_mode,
            Simulation_mode

        }
        enum FixType
        {
            noFix = 1,
            Fix2D,
            Fix3D
        }
        struct SatelliteData
        {
            public int PRN;
            public double elevation;
            public double azimuth;
            public double SNR;
            public SatelliteData(SatelliteVehicle sat)
            {
                PRN = sat.PrnNumber;
                elevation = sat.Elevation;
                azimuth = sat.Azimuth;
                SNR = sat.SignalToNoiseRatio;
            }
        }
        struct SkewData
        {
            public double packetDelay_ms;
            public double skew_ns;
            public Int64 absTime_ticks;
            public PpsStatus status;
        }
        struct SensorData
        {
            public double temperature_bmp180_C;
            public double pressure_Pa;

            public double visibleLightIntensity_lx;
            public double infraredLighIntensity;

            public double temperature_sht21_C;
            public double relativeHumidity_percentage;

            public double sampleAge_ms;

            public SenDataStatus status;
        }
        private class NmeaBlock
        {
            public DateTime time;
            public bool valid;
            public GeoCoordinate coordinates = new GeoCoordinate();
            public FixQuality fixQuality;
            public TimeSpan DGPS_age;
            public int DGPS_ID;
            public int numberOfTrackedSatellites;
            public int numberOfSatellitesInView { get { return numberOfSatellitesInViewGPS + numberOfSatellitesInViewGLONASS + numberOfSatellitesInViewGalileo; } }
            public FixType fixType;
            public double PDOP;
            public double HDOP;
            public double VDOP;

            public int[] usedSatellitesGPS;
            public int numberOfUsedSatellitesGPS { get { return usedSatellitesGPS.Length; } }
            public SatelliteData[] satellitesGPS;
            public int numberOfSatellitesInViewGPS;
            public double maxSnrGPS { get { if (satellitesGPS.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellitesGPS.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Max(); } }
            public double avrSnrGPS { get { if (satellitesGPS.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellitesGPS.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Average(); } }

            public int[] usedSatellitesGLONASS;
            public int numberOfUsedSatellitesGLONASS { get { return usedSatellitesGLONASS.Length; } }
            public SatelliteData[] satellitesGLONASS;
            public int numberOfSatellitesInViewGLONASS;
            public double maxSnrGLONASS { get { if (satellitesGLONASS.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellitesGLONASS.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Max(); } }
            public double avrSnrGLONASS { get { if (satellitesGLONASS.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellitesGLONASS.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Average(); } }

            public int[] usedSatellitesGalileo = new int[0];
            public int numberOfUsedSatellitesGalileo { get { return usedSatellitesGalileo.Length; } }
            public SatelliteData[] satellitesGalileo = new SatelliteData[0];
            public int numberOfSatellitesInViewGalileo = 0;
            public double maxSnrGalileo { get { if (satellitesGalileo.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellitesGalileo.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Max(); } }
            public double avrSnrGalileo { get { if (satellitesGalileo.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellitesGalileo.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Average(); } }

            public double maxSNR { get { if (satellites.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellites.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Max(); } }
            public double avrSnr { get { if (satellites.Count(x => !double.IsNaN(x.SNR)) == 0) return double.NaN; else return satellites.Where(x => !double.IsNaN(x.SNR)).Select(x => x.SNR).Average(); } }

            public int[] usedSatellites
            {
                get
                {
                    int[] all = new int[usedSatellitesGPS.Length + usedSatellitesGLONASS.Length + usedSatellitesGalileo.Length];
                    usedSatellitesGPS.CopyTo(all, 0);
                    usedSatellitesGLONASS.CopyTo(all, usedSatellitesGPS.Length);
                    usedSatellitesGalileo.CopyTo(all, usedSatellitesGPS.Length + usedSatellitesGLONASS.Length);
                    return all;
                }
            }
            public SatelliteData[] satellites
            {
                get
                {
                    SatelliteData[] all = new SatelliteData[numberOfSatellitesInView];
                    satellitesGPS.CopyTo(all, 0);
                    satellitesGLONASS.CopyTo(all, satellitesGPS.Length);
                    satellitesGalileo.CopyTo(all, satellitesGPS.Length + satellitesGLONASS.Length);
                    return all;
                }
            }

            public List<string> raw = new List<string>();

            public SkewData skewData;

            public SensorData SensorData;
        }

        class NmeaTestUnit
        {
            private readonly bool collectSkew;

            private bool init = false;
            private NmeaBlock nmeaBlock = null;
            private List<SatelliteData> satellitesGPS = null;
            private List<SatelliteData> satellitesGLONASS = null;
            private List<SatelliteData> satellitesGalileo = null;
            private int GSAindex = 0;
            private readonly NmeaDevice port;
            private readonly ThreadedLogger logger;
            public readonly string designation;
            public ConcurrentQueue<NmeaBlock> queue;
            public readonly string inputResourceLocator;
            public readonly string boardID;
            public readonly double boardHeight;
            public readonly double boardWidth;
            enum MessageIndex
            {
                RMC,
                GGA,
                GSA,
                PGSV,
                LGSV,
                AGSV
            };
            private bool[] messageChecklist = new bool[6];
            private bool skewReceived;
            private bool sensorDataReceived;
            private bool gnsReceived;

            private readonly ThreadedLogger refTimeLog, checkLog, refErrLog, nmeaDelayLog;

            public NmeaTestUnit(string inputResourceLocator, string rawFile, string rawFile_direct, string errFilePath, string refTimeFile, string refCheckFile, string refErrorFile, string designation, string boardID, double boardHeight, double boardWidth, bool collectSkew)
            {
                this.collectSkew = collectSkew;

                this.inputResourceLocator = inputResourceLocator;
                this.boardID = boardID;
                this.boardHeight = boardHeight;
                this.boardWidth = boardWidth;
                queue = new ConcurrentQueue<NmeaBlock>();
                if (int.TryParse(inputResourceLocator, out int comPortNumber))
                {
#if ACER_1 && DEBUG
                    port = new NmeaDevice(new SerialPort("COM" + inputResourceLocator, BAUD_RATE, Parity.None, 8, StopBits.One), rawFile_direct, errFilePath, designation);
#else
                    port = new NmeaDevice(new SerialPort("COM" + inputResourceLocator, collectSkew ? 115200 : 9600, Parity.None, 8, StopBits.One), rawFile_direct, errFilePath, designation);
#endif
                }
                else
                {
                    port = new NmeaDevice(inputResourceLocator, rawFile_direct, errFilePath, designation);
                    port.FileProcessed += FileProcessed;
                }

                port.MessageReceived += NmeaMessageReceived;
                port.OpenPort();
                logger = new ThreadedLogger(rawFile, "RawLogger: " + designation);
                logger.Start();
                this.designation = designation;
                if (collectSkew)
                {
                    refTimeLog = new ThreadedLogger(refTimeFile, "refTimeLogger: " + designation);
                    refTimeLog.Start();
                    refTimeLog.LogLine("Capture Time (UTC)\tabsTime");

                    checkLog = new ThreadedLogger(refCheckFile, "refCheckLogger: " + designation);
                    checkLog.Start();
                    checkLog.LogLine("Capture Time (UTC)\tskew\tref_absTime\tchk_absTime");

                    refErrLog = new ThreadedLogger(refErrorFile, "refErrorLogger: " + designation);
                    refErrLog.Start();
                    refErrLog.LogLine("Capture Time (UTC)\ttype");

                    nmeaDelayLog = new ThreadedLogger(refErrorFile + ".nmdly", "nmeDelayLogger: " + designation);
                    nmeaDelayLog.Start();
                    nmeaDelayLog.LogLine("Capture Time (UTC)\tdelay");
                }

                InputStreamClosed = false;
            }
            private void NmeaMessageReceived(object sender_, EventArgs args_)
            {
                NmeaDevice device = (NmeaDevice)sender_;
                NmeaMessage message_ = ((NmeaMessageReceivedEventArgs)args_).Message;
                if (!init)
                {
                    if (message_.MessageType == "GPTXT" || message_.MessageType == "GNTXT" || message_.MessageType == "GNZDA")
                    {
                        port.startLogging(); //start direct logging
                        init = true;
                        satellitesGPS = new List<SatelliteData>();
                        satellitesGLONASS = new List<SatelliteData>();
                        satellitesGalileo = new List<SatelliteData>();
                        GSAindex = 0;
                        nmeaBlock = new NmeaBlock();
                        for (int i = 0; i < messageChecklist.Length; i++)
                        {
                            messageChecklist[i] = false;
                        }
                        skewReceived = false;
                        sensorDataReceived = false;
                        gnsReceived = false;
                    }
                    return;
                }
                logger.LogLine(message_.ToString());
                nmeaBlock.raw.Add(message_.ToString());
                switch (message_.MessageType)
                {
                    case "EXTRAREF":
                        if (collectSkew)
                        {
                            string captureTime = DateTime.UtcNow.ToString("yyyy/MM/dd HH:mm:ss");
                            Extraref message = (Extraref)message_;
                            refErrLog.LogLine(stringMaker("\t", captureTime, "EXTRAREF," + message.channelName));
                        }
                        break;
                    case "REFTIME":
                        if (collectSkew)
                        {
                            string captureTime = DateTime.UtcNow.ToString("yyyy/MM/dd HH:mm:ss");
                            Reftime message = (Reftime)message_;
                            refTimeLog.LogLine(stringMaker("\t", captureTime, message.absTime_ticks.ToString()));
                        }
                        break;
                    case "REFCHECK":
                        if (collectSkew)
                        {
                            string captureTime = DateTime.UtcNow.ToString("yyyy/MM/dd HH:mm:ss");
                            Refcheck message = (Refcheck)message_;

                            switch (message.status)
                            {
                                case PpsStatus.OK:
                                    checkLog.LogLine(stringMaker("\t", captureTime, message.skew_ns.ToString(), message.absTime_ref_ticks.ToString(), message.absTime_check_ticks.ToString()));
                                    break;

                                case PpsStatus.notPossible:
                                    refErrLog.LogLine(stringMaker("\t", captureTime, "not possible"));
                                    break;

                                case PpsStatus.glitch:
                                    refErrLog.LogLine(stringMaker("\t", captureTime, "glitch"));
                                    break;

                                case PpsStatus.old:
                                    refErrLog.LogLine(stringMaker("\t", captureTime, "check is too old"));
                                    checkLog.LogLine(stringMaker("\t", captureTime, "", message.absTime_ref_ticks.ToString(), message.absTime_check_ticks.ToString()));
                                    break;

                                case PpsStatus.ppsMissing:
                                    refErrLog.LogLine(stringMaker("\t", captureTime, "check is missing"));
                                    break;

                                case PpsStatus.refMissing:
                                    refErrLog.LogLine(stringMaker("\t", captureTime, "ref is missing"));
                                    checkLog.LogLine(stringMaker("\t", captureTime, "", "", message.absTime_check_ticks.ToString()));
                                    break;

                                case PpsStatus.unexpected:
                                    refErrLog.LogLine(stringMaker("\t", captureTime, "unexpected"));
                                    break;

                                default:
                                    break;
                            }
                        }
                        break;
                    case "SKEW":
                        {
                            if (!collectSkew)
                                break;

                            Skew message = (Skew)message_;
                            nmeaBlock.skewData.packetDelay_ms = message.packetDelay;
                            nmeaBlock.skewData.skew_ns = message.skew_ns;
                            nmeaBlock.skewData.absTime_ticks = message.absTime_ticks;
                            nmeaBlock.skewData.status = message.status;
                            skewReceived = true;

                            nmeaDelayLog.LogLine("{0}\t{1}", DateTime.UtcNow.ToString("yyyy/MM/dd HH:mm:ss"), message.packetDelay);
                        }
                        break;
                    case "SENDAT":
                        {
                            SenData message = (SenData)message_;
                            nmeaBlock.SensorData.temperature_bmp180_C = message.temperature_bmp180_C;
                            nmeaBlock.SensorData.pressure_Pa = message.pressure_Pa;
                            nmeaBlock.SensorData.visibleLightIntensity_lx = message.visibleLightIntensity_lx;
                            nmeaBlock.SensorData.infraredLighIntensity = message.infraredLighIntensity;
                            nmeaBlock.SensorData.temperature_sht21_C = message.temperature_sht21_C;
                            nmeaBlock.SensorData.relativeHumidity_percentage = message.relativeHumidity_percentage;

                            nmeaBlock.SensorData.sampleAge_ms = message.sampleAge_ms;
                            nmeaBlock.SensorData.status = message.status;

                            sensorDataReceived = true;
                        }
                        break;
                    case "GPRMC":
                        {
                            Gprmc message = (Gprmc)message_;
                            nmeaBlock.time = message.FixTime;
                            nmeaBlock.valid = message.Active;
                            nmeaBlock.coordinates.Latitude = message.Latitude;
                            nmeaBlock.coordinates.Longitude = message.Longitude;
                            nmeaBlock.coordinates.Speed = (message.Speed < 0) ? double.NaN : (message.Speed * 0.51444444444);
                            nmeaBlock.coordinates.Course = message.Course;
                            messageChecklist[(int)MessageIndex.RMC] = true;
                        }
                        break;
                    case "GNRMC":
                        {
                            Gnrmc message = (Gnrmc)message_;
                            nmeaBlock.time = message.FixTime;
                            nmeaBlock.valid = message.Active;
                            nmeaBlock.coordinates.Latitude = message.Latitude;
                            nmeaBlock.coordinates.Longitude = message.Longitude;
                            nmeaBlock.coordinates.Speed = (message.Speed < 0) ? double.NaN : (message.Speed * 0.51444444444);
                            nmeaBlock.coordinates.Course = message.Course;
                            messageChecklist[(int)MessageIndex.RMC] = true;
                        }
                        break;
                    case "GPGGA":
                        {
                            Gpgga message = (Gpgga)message_;
                            nmeaBlock.fixQuality = (FixQuality)message.Quality;
                            if (!gnsReceived)
                                nmeaBlock.numberOfTrackedSatellites = message.NumberOfSatellites;
                            nmeaBlock.coordinates.Altitude = message.Altitude;
                            nmeaBlock.DGPS_age = message.TimeSinceLastDgpsUpdate;
                            nmeaBlock.DGPS_ID = message.DgpsStationId;
                            messageChecklist[(int)MessageIndex.GGA] = true;
                        }
                        break;
                    case "GNGGA":
                        {
                            Gngga message = (Gngga)message_;
                            nmeaBlock.fixQuality = (FixQuality)message.Quality;
                            if (!gnsReceived)
                                nmeaBlock.numberOfTrackedSatellites = message.NumberOfSatellites;
                            nmeaBlock.coordinates.Altitude = message.Altitude;
                            nmeaBlock.DGPS_age = message.TimeSinceLastDgpsUpdate;
                            nmeaBlock.DGPS_ID = message.DgpsStationId;
                            messageChecklist[(int)MessageIndex.GGA] = true;
                        }
                        break;
                    case "GPGSA":
                        {
                            Gpgsa message = (Gpgsa)message_;
                            nmeaBlock.fixType = (FixType)message.FixMode;
                            nmeaBlock.HDOP = message.Hdop;
                            nmeaBlock.VDOP = message.Vdop;
                            nmeaBlock.PDOP = message.Pdop;

                            GSAindex++;
                            switch (GSAindex)
                            {
                                case 1:
                                    nmeaBlock.usedSatellitesGPS = message.SVs.OrderBy((x) => x).ToArray();
                                    break;

                                case 2:
                                    nmeaBlock.usedSatellitesGLONASS = message.SVs.OrderBy((x) => x).ToArray();
                                    break;

                                case 3:
                                    nmeaBlock.usedSatellitesGalileo = message.SVs.OrderBy((x) => x).ToArray();
                                    break;
                            }

                            messageChecklist[(int)MessageIndex.GSA] = true;
                        }
                        break;
                    case "GNGSA":
                        {
                            Gngsa message = (Gngsa)message_;
                            nmeaBlock.fixType = (FixType)message.FixMode;
                            nmeaBlock.HDOP = message.Hdop;
                            nmeaBlock.VDOP = message.Vdop;
                            nmeaBlock.PDOP = message.Pdop;

                            GSAindex++;
                            switch (GSAindex)
                            {
                                case 1:
                                    nmeaBlock.usedSatellitesGPS = message.SVs.OrderBy((x) => x).ToArray();
                                    break;

                                case 2:
                                    nmeaBlock.usedSatellitesGLONASS = message.SVs.OrderBy((x) => x).ToArray();
                                    break;

                                case 3:
                                    nmeaBlock.usedSatellitesGalileo = message.SVs.OrderBy((x) => x).ToArray();
                                    break;
                            }

                            messageChecklist[(int)MessageIndex.GSA] = true;
                        }
                        break;
                    case "GPGSV":
                        {
                            Gpgsv message = (Gpgsv)message_;
                            nmeaBlock.numberOfSatellitesInViewGPS = message.SVsInView;
                            foreach (SatelliteVehicle sat in message.SVs)
                                satellitesGPS.Add(new SatelliteData(sat));

                            messageChecklist[(int)MessageIndex.PGSV] = true;
                        }
                        break;
                    case "GLGSV":
                        {
                            Glgsv message = (Glgsv)message_;
                            nmeaBlock.numberOfSatellitesInViewGLONASS = message.SVsInView;
                            foreach (SatelliteVehicle sat in message.SVs)
                                satellitesGLONASS.Add(new SatelliteData(sat));

                            messageChecklist[(int)MessageIndex.LGSV] = true;
                        }
                        break;
                    case "GAGSV":
                        {
                            Gagsv message = (Gagsv)message_;
                            nmeaBlock.numberOfSatellitesInViewGalileo = message.SVsInView;
                            foreach (SatelliteVehicle sat in message.SVs)
                                satellitesGalileo.Add(new SatelliteData(sat));

                            messageChecklist[(int)MessageIndex.AGSV] = true;
                        }
                        break;
                    case "GPGNS":
                        {
                            Gpgns message = (Gpgns)message_;

                            gnsReceived = true;
                            nmeaBlock.numberOfTrackedSatellites = message.numberOfUsedSatellites;
                        }
                        break;
                    case "GNGNS":
                        {
                            Gngns message = (Gngns)message_;

                            gnsReceived = true;
                            nmeaBlock.numberOfTrackedSatellites = message.numberOfUsedSatellites;
                        }
                        break;
                    case "GNZDA":
                        if (!gnsReceived && nmeaBlock.numberOfTrackedSatellites == 12 && GSAindex >= 2)
                            nmeaBlock.numberOfTrackedSatellites = nmeaBlock.numberOfUsedSatellitesGLONASS + nmeaBlock.numberOfUsedSatellitesGPS + nmeaBlock.numberOfUsedSatellitesGalileo;
                        goto case "GPTXT";
                    case "GPTXT":
                        nmeaBlock.satellitesGPS = satellitesGPS.OrderBy((x) => x.PRN).ToArray();
                        nmeaBlock.satellitesGLONASS = satellitesGLONASS.OrderBy((x) => x.PRN).ToArray();
                        nmeaBlock.satellitesGalileo = satellitesGalileo.OrderBy((x) => x.PRN).ToArray();

                        if (((!messageChecklist.Take(5).Contains(false) && GSAindex == 2) || (!messageChecklist.Contains(false) && GSAindex == 3)) && (!collectSkew || (skewReceived && sensorDataReceived)))
                        {
                            if (nmeaBlock.satellitesGPS.Length == nmeaBlock.numberOfSatellitesInViewGPS && nmeaBlock.satellitesGLONASS.Length == nmeaBlock.numberOfSatellitesInViewGLONASS && nmeaBlock.satellitesGalileo.Length == nmeaBlock.numberOfSatellitesInViewGalileo)
                                queue.Enqueue(nmeaBlock);
                            else
                            {
                                nmeaBlock.numberOfSatellitesInViewGPS = 0;
                                nmeaBlock.numberOfSatellitesInViewGLONASS = 0;
                                nmeaBlock.numberOfSatellitesInViewGalileo = 0;
                                nmeaBlock.satellitesGLONASS = new List<SatelliteData>().ToArray();
                                nmeaBlock.satellitesGPS = new List<SatelliteData>().ToArray();
                                nmeaBlock.satellitesGalileo = new List<SatelliteData>().ToArray();
                                nmeaBlock.numberOfTrackedSatellites = 0;
                                nmeaBlock.usedSatellitesGLONASS = new int[0];
                                nmeaBlock.usedSatellitesGPS = new int[0];
                                nmeaBlock.usedSatellitesGalileo = new int[0];
                                nmeaBlock.valid = false;
                                queue.Enqueue(nmeaBlock);
                            }
                        }

                        satellitesGPS = new List<SatelliteData>();
                        satellitesGLONASS = new List<SatelliteData>();
                        satellitesGalileo = new List<SatelliteData>();
                        nmeaBlock = new NmeaBlock();
                        GSAindex = 0;
                        for (int i = 0; i < messageChecklist.Length; i++)
                        {
                            messageChecklist[i] = false;
                        }
                        skewReceived = false;
                        sensorDataReceived = false;
                        gnsReceived = false;
                        break;
                    case "GNTXT":
                        nmeaBlock.satellitesGPS = satellitesGPS.ToArray();
                        nmeaBlock.satellitesGLONASS = satellitesGLONASS.ToArray();

                        if (((!messageChecklist.Take(5).Contains(false) && GSAindex == 2) || (!messageChecklist.Contains(false) && GSAindex == 3)) && (!collectSkew || (skewReceived && sensorDataReceived)))
                            if (nmeaBlock.satellitesGPS.Length == nmeaBlock.numberOfSatellitesInViewGPS && nmeaBlock.satellitesGLONASS.Length == nmeaBlock.numberOfSatellitesInViewGLONASS && nmeaBlock.satellitesGalileo.Length == nmeaBlock.numberOfSatellitesInViewGalileo)
                                queue.Enqueue(nmeaBlock);
                            else
                            {
                                nmeaBlock.numberOfSatellitesInViewGPS = 0;
                                nmeaBlock.numberOfSatellitesInViewGLONASS = 0;
                                nmeaBlock.numberOfSatellitesInViewGalileo = 0;
                                nmeaBlock.satellitesGLONASS = new List<SatelliteData>().ToArray();
                                nmeaBlock.satellitesGPS = new List<SatelliteData>().ToArray();
                                nmeaBlock.satellitesGalileo = new List<SatelliteData>().ToArray();
                                nmeaBlock.numberOfTrackedSatellites = 0;
                                nmeaBlock.usedSatellitesGLONASS = new int[0];
                                nmeaBlock.usedSatellitesGPS = new int[0];
                                nmeaBlock.usedSatellitesGalileo = new int[0];
                                nmeaBlock.valid = false;
                                queue.Enqueue(nmeaBlock);
                            }

                        satellitesGPS = new List<SatelliteData>();
                        satellitesGLONASS = new List<SatelliteData>();
                        satellitesGalileo = new List<SatelliteData>();
                        nmeaBlock = new NmeaBlock();
                        GSAindex = 0;
                        for (int i = 0; i < messageChecklist.Length; i++)
                        {
                            messageChecklist[i] = false;
                        }
                        skewReceived = false;
                        sensorDataReceived = false;
                        gnsReceived = false;
                        break;
                    default:
                        break;
                }
            }

            private void FileProcessed(object sender, EventArgs args)
            {
                InputStreamClosed = true;
            }

            public bool Paused
            {
                get
                {
                    bool paused = false;

                    if (collectSkew)
                    {
                        paused |= refTimeLog.Paused;
                        paused |= checkLog.Paused;
                        paused |= refErrLog.Paused;
                        paused |= nmeaDelayLog.Paused;
                    }

                    return paused;
                }
            }
            public void PauseLoggers(TimeSpan time)
            {
                logger.Pause(time);
                if (collectSkew)
                {
                    refTimeLog.Pause(time);
                    checkLog.Pause(time);
                    refErrLog.Pause(time);

                    nmeaDelayLog.Pause(time);
                }
            }

            public void ResumeLoggers()
            {
                logger.ReStart();
                if (collectSkew)
                {
                    refTimeLog.ReStart();
                    checkLog.ReStart();
                    refErrLog.ReStart();

                    nmeaDelayLog.ReStart();
                }
            }

            public bool InputStreamClosed { private set; get; }
            public void Close()
            {
                port.MessageReceived -= NmeaMessageReceived;
                logger.Close();
                if (collectSkew)
                {
                    refTimeLog.Close();
                    checkLog.Close();
                    refErrLog.Close();

                    nmeaDelayLog.Close();
                }

                port.Close();
            }

            public void ResetInputStream()
            {
                NmeaBlock trash;
                while (!queue.IsEmpty)
                    while (!queue.TryDequeue(out trash))
                        Thread.Yield();

                port.ResetInputStream();
            }
        }

        static NmeaTestUnit[] nmeaTestUnits;

        static void Main(string[] args)
        {
            #region start
            SetConsoleCtrlHandler(terminationEventHandler, true);

            Console.Write("Do you wish to use a setup file? (Y/N)");
            string isConUsed = Console.ReadLine().ToLower();


            Console.Write("Label of measurement: ");
            string lable = Console.ReadLine();
#if DEBUG
            bool deleteFile;
            if (lable.Length > 2 && lable.StartsWith("__"))
            {
                deleteFile = true;
                lable = lable.Substring(2);
            }
            else
                deleteFile = false;
#endif

            Console.Write("Work directory: ");
#if DEBUG
#if ACER_1
            string folder = @"C:\Users\Adam\Desktop\GND Size Study\";
#elif DELL_2
            string folder = @"C:\Users\Adam-MIT\Desktop\Measurements\";
#elif MIT_PC_1
            string folder = @"C:\Users\hollos\Desktop\Skew Measurement\";
#elif DELL_SERVER_1
            string folder = @"C:\Users\Administrator\Desktop\Measurements";
#else
            string folder = Console.ReadLine();
#endif
            Console.WriteLine(folder);
#else
            string folder = Console.ReadLine();
#endif
            string workFolder = Path.Combine(folder, lable) + @"\";
            if (Directory.Exists(workFolder))
            {
                Console.WriteLine("A measurement with this lable already exists in this directory!");
#if DEBUG
                if (deleteFile)
                {
                    Directory.Delete(workFolder, true);
                }
                else
                {
                    Console.WriteLine("Press C for continue (the contents of the folder will be deleted) or Q to quit.");
                    string chIn = Console.ReadLine();
                    if (chIn == "C" || chIn == "c")
                        Directory.Delete(workFolder, true);
                    else
                        Environment.Exit(0);
                }

#else
                Environment.Exit(0);
#endif
            }
            try
            {
                Directory.CreateDirectory(workFolder);
            }
            catch (Exception e)
            {
                Console.WriteLine("Bad directory!");
                Console.ReadLine();
                Environment.Exit(0);
            }
            Task.Run(() => logTime(workFolder));

            string DAQ_ID;
            bool collectSkew = false, ppsCardUsed = false;
            PpsCard ppsCard = null;
            if (isConUsed == "y")
            {
                Console.Write("Path of setup file: ");
#if DEBUG
#if ACER_1
                string setUpFilePath = @"C:\Users\Adam\Desktop\GND Size Study\i2mtc_I_mod\i2mtc_I_mod_rerun.xml";
#elif MIT_PC_1
                string setUpFilePath = @"C:\Users\hollos\Desktop\Skew Measurement\Setup.xml";
#elif DELL_SERVER_1
                string setUpFilePath = @"C:\Users\Administrator\Desktop\Measurements\setup.xml";
#else
                string setUpFilePath = Console.ReadLine();
#endif
#else
                string setUpFilePath = Console.ReadLine();
#endif
                XDocument setUpFile;
                try
                {
                    setUpFile = XDocument.Load(setUpFilePath);
                    DAQ_ID = setUpFile.Root.Element("DAQ_Device").Element("ID").Value;
                    string measurementType = setUpFile.Root.Element("measurementType").Value;
                    switch (measurementType)
                    {
                        case "withSkew":
                            collectSkew = true;
                            break;

                        case "simple":
                        default:
                            collectSkew = false;
                            break;
                    }

                    var ppsDev = from x in setUpFile.Root.Element("devices").Elements("device")
                                 where x.Attribute("type").Value == "PPS card"
                                 select x;

                    if (ppsDev.Count() == 1)
                    {
                        try
                        {
                            ppsCard = new PpsCard(ppsDev.First().Element("COM_portNumber").Value,
                                                  workFolder + lable + "_ref.dly",
                                                  workFolder + lable + "_ref_delay_Direct.raw",
                                                  workFolder + lable + "_ref_delay.parerr");
                        }
                        catch
                        {
                            Console.WriteLine("PPS card COM port error!");
                            Console.ReadLine();
                            Environment.Exit(0);
                        }

                        ppsCardUsed = true;
                    }
                    else if (ppsDev.Count() > 1)
                    {
                        throw new Exception("more than one PPS card");
                    }

                    List<NmeaTestUnit> nmeaTestUnitsList = new List<NmeaTestUnit>();

                    var refDev = (from x in setUpFile.Root.Element("devices").Elements("device")
                                  where x.Attribute("type").Value == "NMEA Test Unit" && x.Attribute("role").Value == "reference"
                                  select x).First();
                    nmeaTestUnitsList.Add(new NmeaTestUnit(refDev.Element("COM_portNumber").Value, //portNum
                                            workFolder + lable + "_" + refDev.Element("designation").Value + "_REF.raw",
                                            workFolder + lable + "_" + refDev.Element("designation").Value + "_REF_Direct.raw",
                                            workFolder + lable + "_" + refDev.Element("designation").Value + "_REF.parerr",
                                            workFolder + lable + "_" + refDev.Element("designation").Value + "_REF.ref",
                                            workFolder + lable + "_" + refDev.Element("designation").Value + "_REF.chk",
                                            workFolder + lable + "_" + refDev.Element("designation").Value + "_REF_CHECK.err",
                                            refDev.Element("designation").Value, //designation
                                            refDev.Element("board").Element("ID").Value, //boardID
                                            Convert.ToDouble(refDev.Element("board").Element("height").Value), //W
                                            Convert.ToDouble(refDev.Element("board").Element("width").Value),
                                            collectSkew));

                    var devs = from x in setUpFile.Root.Element("devices").Elements("device")
                               where x.Attribute("type").Value == "NMEA Test Unit" && x.Attribute("role").Value == "DUT"
                               select x;
                    foreach (var unit in devs)
                    {
                        nmeaTestUnitsList.Add(new NmeaTestUnit(unit.Element("COM_portNumber").Value, //portNum
                                            workFolder + lable + "_" + unit.Element("designation").Value + "_DUT.raw",
                                            workFolder + lable + "_" + unit.Element("designation").Value + "_DUT_Direct.raw",
                                            workFolder + lable + "_" + unit.Element("designation").Value + "_DUT.parerr",
                                            workFolder + lable + "_" + unit.Element("designation").Value + "_DUT.ref",
                                            workFolder + lable + "_" + unit.Element("designation").Value + "_DUT.chk",
                                            workFolder + lable + "_" + unit.Element("designation").Value + "_DUT_CHECK.err",
                                            unit.Element("designation").Value, //designation
                                            unit.Element("board").Element("ID").Value, //boardID
                                            Convert.ToDouble(unit.Element("board").Element("height").Value), //W
                                            Convert.ToDouble(unit.Element("board").Element("width").Value),
                                            collectSkew));
                    }
                    nmeaTestUnits = nmeaTestUnitsList.ToArray();
                }
                catch (Exception e)
                {
                    Console.WriteLine("Bad setup file!");
                    Console.ReadLine();
                    Environment.Exit(0);
                    while (true) ; //to prevent the compiler from crying beacuse DAQ_ID is not initialized
                }
            }
            else
            {

                Console.WriteLine("Does the setup incorporate a PPS card? (Y/N)");
                string ppsCard_str = Console.ReadLine().ToLower();
                if (ppsCard_str == "y")
                {
                    Console.Write("Specify the COM port number of the PPS card: ");
                    string ppsCard_com = Console.ReadLine();

                    try
                    {
                        ppsCard = new PpsCard(ppsCard_com,
                                              workFolder + lable + "_ref.dly",
                                              workFolder + lable + "_ref_delay_Direct.raw",
                                              workFolder + lable + "_ref_delay.parerr");
                    }
                    catch
                    {
                        Console.WriteLine("PPS card COM port error!");
                        Console.ReadLine();
                        Environment.Exit(0);
                    }

                    ppsCardUsed = true;
                }

                Console.WriteLine("Does the setup incorporate skew measurement? (Y/N)");
                string collectSkew_str = Console.ReadLine().ToLower();
                if (collectSkew_str == "y")
                    collectSkew = true;


                Console.Write("data acquisition device ID: ");
                DAQ_ID = Console.ReadLine();

                Console.WriteLine();
                Console.WriteLine(">>>Device 0<<< (REF)");
                List<NmeaTestUnit> nmeaTestUnitsList = new List<NmeaTestUnit>();
                Console.Write("reference device designation: ");
                string ref_des = Console.ReadLine();
                Console.Write("reference device board ID: ");
                string ref_boardID = Console.ReadLine();
                double ref_boardHeight;
                while (true)
                {
                    Console.Write("reference device board height [mm]: ");
                    try
                    {
                        ref_boardHeight = Convert.ToDouble(Console.ReadLine());
                        break;
                    }
                    catch (Exception)
                    {
                        Console.WriteLine("Invalid height!");
                    }
                }
                double ref_boardWidth;
                while (true)
                {
                    Console.Write("reference device board width [mm]: ");
                    try
                    {
                        ref_boardWidth = Convert.ToDouble(Console.ReadLine());
                        break;
                    }
                    catch (Exception)
                    {
                        Console.WriteLine("Invalid width!");
                    }
                }
                Console.Write("reference device COM port number: ");
                string ref_com = Console.ReadLine();
                try
                {
                    nmeaTestUnitsList.Add(new NmeaTestUnit(ref_com,
                                                           workFolder + lable + "_" + ref_des + "_REF.raw",
                                                           workFolder + lable + "_" + ref_des + "_REF_Direct.raw",
                                                           workFolder + lable + "_" + ref_des + "_REF.parerr",
                                                           workFolder + lable + "_" + ref_des + "_REF.ref",
                                                           workFolder + lable + "_" + ref_des + "_REF.chk",
                                                           workFolder + lable + "_" + ref_des + "_REF_CHECK.err",
                                                           ref_des,
                                                           ref_boardID,
                                                           ref_boardHeight,
                                                           ref_boardWidth,
                                                           collectSkew));
                }
                catch (Exception)
                {
                    Console.WriteLine("COM port error!");
                    Console.ReadLine();
                    Environment.Exit(0);
                }

                int deviceSerialID = 0;
                do
                {
                    deviceSerialID++;
                    Console.WriteLine();
                    Console.WriteLine(">>>Device {0}<<<", deviceSerialID);
                    Console.Write("device designation: ");
                    string des = Console.ReadLine();
                    Console.Write("device board ID: ");
                    string boardID = Console.ReadLine();
                    double boardHeight;
                    while (true)
                    {
                        Console.Write("device board height [mm]: ");
                        try
                        {
                            boardHeight = Convert.ToDouble(Console.ReadLine());
                            break;
                        }
                        catch (Exception)
                        {
                            Console.WriteLine("Invalid height!");
                        }
                    }
                    double boardWidth;
                    while (true)
                    {
                        Console.Write("device board width [mm]: ");
                        try
                        {
                            boardWidth = Convert.ToDouble(Console.ReadLine());
                            break;
                        }
                        catch (Exception)
                        {
                            Console.WriteLine("Invalid width!");
                        }
                    }
                    Console.Write("device COM port number: ");
                    string comNum = Console.ReadLine();
                    try
                    {
                        nmeaTestUnitsList.Add(new NmeaTestUnit(comNum,
                                                               workFolder + lable + "_" + des + "_DUT.raw",
                                                               workFolder + lable + "_" + des + "_DUT_Direct.raw",
                                                               workFolder + lable + "_" + des + "_DUT.parerr",
                                                               workFolder + lable + "_" + des + "_DUT.ref",
                                                               workFolder + lable + "_" + des + "_DUT.chk",
                                                               workFolder + lable + "_" + des + "_DUT_CHECK.err",
                                                               des,
                                                               boardID,
                                                               boardHeight,
                                                               boardWidth,
                                                               collectSkew));
                    }
                    catch (Exception)
                    {
                        Console.WriteLine("COM port error!");
                        Console.ReadLine();
                        Environment.Exit(0);
                    }

                    Console.WriteLine("Do you want to add an onther device? (Y/N)");
                } while (Console.ReadLine().ToLower() == "y");
                nmeaTestUnits = nmeaTestUnitsList.ToArray();
            }
            try
            {
                using (FileStream setupFile = File.Open(workFolder + lable + @"_Setup.xml", FileMode.Create))
                using (StreamWriter twriter = new StreamWriter(setupFile, Encoding.UTF8))
                using (XmlTextWriter writer = new XmlTextWriter(twriter))
                {
                    writer.Formatting = Formatting.Indented;
                    writer.IndentChar = '\t';
                    writer.Indentation = 1;

                    writer.WriteStartElement("setup");
                    {
                        writer.WriteStartElement("measurementType");
                        {
                            if (collectSkew)
                                writer.WriteString("withSkew");
                            else
                                writer.WriteString("simple");
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("SNR_Deviance_unit");
                        {
                            writer.WriteString("dB");
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("DAQ_Device");
                        {
                            writer.WriteStartElement("ID");
                            {
                                writer.WriteString(DAQ_ID);
                            }
                            writer.WriteEndElement();
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("DAQ_Software");
                        {
                            writer.WriteStartElement("version");
                            {
                                writer.WriteString(SoftwareVersion);
                            }
                            writer.WriteEndElement();

                            writer.WriteStartElement("buildeType");
                            {
#if DEBUG
                                writer.WriteString("DEBUG");
#else
                                writer.WriteString("RELEASE");
#endif
                            }
                            writer.WriteEndElement();

                            writer.WriteStartElement("hash");
                            {
                                writer.WriteString(GetMD5());
                            }
                            writer.WriteEndElement();
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("devices");
                        {
                            if (ppsCardUsed)
                            {
                                writer.WriteStartElement("device");
                                {
                                    writer.WriteAttributeString("type", "PPS card");

                                    writer.WriteStartElement("COM_portNumber");
                                    {
                                        writer.WriteString(ppsCard.InputResourceLocator);
                                    }
                                    writer.WriteEndElement();
                                }
                                writer.WriteEndElement();
                            }

                            writer.WriteStartElement("device");
                            {
                                writer.WriteAttributeString("role", "reference");
                                writer.WriteAttributeString("type", "NMEA Test Unit");
                                writer.WriteStartElement("designation");
                                {
                                    writer.WriteString(nmeaTestUnits[0].designation);
                                }
                                writer.WriteEndElement();

                                writer.WriteStartElement("device_ID");
                                {
                                    writer.WriteString("0");
                                }
                                writer.WriteEndElement();

                                writer.WriteStartElement("COM_portNumber");
                                {
                                    writer.WriteString(nmeaTestUnits[0].inputResourceLocator);
                                }
                                writer.WriteEndElement();

                                writer.WriteStartElement("board");
                                {
                                    writer.WriteStartElement("ID");
                                    {
                                        writer.WriteString(nmeaTestUnits[0].boardID);
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("height");
                                    {
                                        writer.WriteAttributeString("unit", "mm");
                                        writer.WriteString(nmeaTestUnits[0].boardHeight.ToString());
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("width");
                                    {
                                        writer.WriteAttributeString("unit", "mm");
                                        writer.WriteString(nmeaTestUnits[0].boardWidth.ToString());
                                    }
                                    writer.WriteEndElement();
                                }
                                writer.WriteEndElement();
                            }
                            writer.WriteEndElement();

                            for (int i = 1; i < nmeaTestUnits.Length; i++)
                            {
                                writer.WriteStartElement("device");
                                {
                                    writer.WriteAttributeString("role", "DUT");
                                    writer.WriteAttributeString("type", "NMEA Test Unit");
                                    writer.WriteStartElement("designation");
                                    {
                                        writer.WriteString(nmeaTestUnits[i].designation);
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("device_ID");
                                    {
                                        writer.WriteString(i.ToString());
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("COM_portNumber");
                                    {
                                        writer.WriteString(nmeaTestUnits[i].inputResourceLocator);
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("board");
                                    {
                                        writer.WriteStartElement("ID");
                                        {
                                            writer.WriteString(nmeaTestUnits[i].boardID);
                                        }
                                        writer.WriteEndElement();

                                        writer.WriteStartElement("height");
                                        {
                                            writer.WriteAttributeString("unit", "mm");
                                            writer.WriteString(nmeaTestUnits[i].boardHeight.ToString());
                                        }
                                        writer.WriteEndElement();

                                        writer.WriteStartElement("width");
                                        {
                                            writer.WriteAttributeString("unit", "mm");
                                            writer.WriteString(nmeaTestUnits[i].boardWidth.ToString());
                                        }
                                        writer.WriteEndElement();
                                    }
                                    writer.WriteEndElement();
                                }
                                writer.WriteEndElement();
                            }
                        }
                        writer.WriteEndElement();
                    }
                    writer.WriteEndElement();
                }
            }
            catch (Exception)
            {
                Console.WriteLine("Bad lable!");
                Console.ReadLine();
                Environment.Exit(0);
            }

            ThreadedLogger devLog;
            try
            {
                devLog = new ThreadedLogger(workFolder + lable + ".dev", "devLog");
                devLog.Start();

                devLog.Log("time (UTC)");
                for (int i = 1; i < nmeaTestUnits.Length; i++)
                {
                    devLog.Log("\t{0}_offset\t{0}_PDOP\t{0}_HDOP\t{0}_VDOP\t{0}_satelliteInView\t{0}_satelliteInViewGPS\t{0}_satelliteInViewGLONASS\t{0}_satelliteUsed\t{0}_satelliteUsedGPS\t{0}_satelliteUsedGLONASS\t{0}_maxSnr\t{0}_maxSnrGPS\t{0}_maxSnrGLONASS\t{0}_avrSnr\t{0}_avrSnrGPS\t{0}_avrSnrGLONASS\t{0}_SnrDevView\t{0}_SnrDevUsed", nmeaTestUnits[i].designation);
                }
                devLog.LogLine();
            }
            catch (Exception)
            {
                Console.WriteLine(".dev file init error!");
                Console.ReadLine();
                Environment.Exit(0);
                while (true) ; //to prevent the compiler from crying beacuse devLog is not initialized
            }

            ThreadedLogger datLog;
            try
            {
                datLog = new ThreadedLogger(workFolder + lable + ".dat", "datLog");
                datLog.Start();

                datLog.Log("time (UTC)");
                for (int i = 0; i < nmeaTestUnits.Length; i++)
                {
                    datLog.Log("\t{0}_PDOP\t{0}_HDOP\t{0}_VDOP\t{0}_satelliteInView\t{0}_satelliteInViewGPS\t{0}_satelliteInViewGLONASS\t{0}_satelliteUsed\t{0}_satelliteUsedGPS\t{0}_satelliteUsedGLONASS\t{0}_maxSnr\t{0}_maxSnrGPS\t{0}_maxSnrGLONASS\t{0}_avrSnr\t{0}_avrSnrGPS\t{0}_avrSnrGLONASS", nmeaTestUnits[i].designation);
                }
                datLog.LogLine();

            }
            catch (Exception)
            {
                Console.WriteLine(".dat file init error!");
                Console.ReadLine();
                Environment.Exit(0);
                while (true) ; //to prevent the compiler from crying beacuse datLog is not initialized
            }

            ThreadedLogger errLog;
            try
            {
                errLog = new ThreadedLogger(workFolder + lable + ".err", "errLog", 1024 * 500);
                errLog.Start();

                errLog.LogLine("time (UTC)\tdevice\ttype\tcode");
            }
            catch (Exception)
            {
                Console.WriteLine(".err file init error!");
                Console.ReadLine();
                Environment.Exit(0);
                while (true) ; //to prevent the compiler from crying beacuse errLog is not initialized
            }

            ThreadedLogger locLog;
            try
            {
                locLog = new ThreadedLogger(workFolder + lable + ".loc", "locLog");
                locLog.Start();


                locLog.Log("time (UTC)");
                for (int i = 0; i < nmeaTestUnits.Length; i++)
                {
                    locLog.Log("\t{0}_latitude\t{0}_longitude\t{0}_altitude", nmeaTestUnits[i].designation);
                }
                locLog.LogLine();
            }
            catch (Exception)
            {
                Console.WriteLine(".loc file init error!");
                Console.ReadLine();
                Environment.Exit(0);
                while (true) ; //to prevent the compiler from crying beacuse locLog is not initialized
            }

            ThreadedLogger skewLog = null, skewErrLog = null, senLog = null, senErrLog = null;
            if (collectSkew)
            {
                try
                {
                    skewLog = new ThreadedLogger(workFolder + lable + ".skw", "skewLog");
                    skewLog.Start();


                    skewLog.Log("time (UTC)");
                    for (int i = 0; i < nmeaTestUnits.Length; i++)
                    {
                        skewLog.Log("\t{0}_skew\t{0}_absTime", nmeaTestUnits[i].designation);
                    }
                    skewLog.LogLine();
                }
                catch (Exception)
                {
                    Console.WriteLine(".skw file init error!");
                    Console.ReadLine();
                    Environment.Exit(0);
                    while (true) ; //to prevent the compiler from crying beacuse skwLog is not initialized
                }
                try
                {
                    skewErrLog = new ThreadedLogger(workFolder + lable + "_SKEW.err", "skewErrLog");
                    skewErrLog.Start();


                    skewErrLog.LogLine("time (UTC)\tdevice\ttype");
                }
                catch (Exception)
                {
                    Console.WriteLine("_SKEW.err file init error!");
                    Console.ReadLine();
                    Environment.Exit(0);
                    while (true) ; //to prevent the compiler from crying beacuse skwErrLog is not initialized
                }

                try
                {
                    senLog = new ThreadedLogger(workFolder + lable + ".sen", "senLog");
                    senLog.Start();


                    senLog.Log("time (UTC)");
                    for (int i = 0; i < nmeaTestUnits.Length; i++)
                    {
                        senLog.Log("\t{0}_tempBmp180\t{0}_pressure\t{0}_visibleLight\t{0}_infraredLight\t{0}_tempSht21\t{0}_relativeHumidity", nmeaTestUnits[i].designation);
                    }
                    senLog.LogLine();
                }
                catch (Exception)
                {
                    Console.WriteLine(".sen file init error!");
                    Console.ReadLine();
                    Environment.Exit(0);
                    while (true) ; //to prevent the compiler from crying beacuse senLog is not initialized
                }
                try
                {
                    senErrLog = new ThreadedLogger(workFolder + lable + "_SEN.err", "skewErrLog");
                    senErrLog.Start();

                    senErrLog.LogLine("time (UTC)\tdevice\ttype");
                }
                catch (Exception)
                {
                    Console.WriteLine("_SEN.err file init error!");
                    Console.ReadLine();
                    Environment.Exit(0);
                    while (true) ; //to prevent the compiler from crying beacuse senErrLog is not initialized
                }
            }

            //empty queues or start reading input files
            foreach (NmeaTestUnit unit in nmeaTestUnits)
            {
                unit.ResetInputStream();
            }
            running = true;
            DateTime startTime = DateTime.UtcNow;
            #endregion

            #region new
            DateTime resumeTime = DateTime.UtcNow;
            DateTime lastScreenUpdateTime = DateTime.UtcNow.Subtract(new TimeSpan(1, 0, 0));

            ThreadedLogger logLog;
            logLog = new ThreadedLogger(workFolder + lable + ".log", "logLog", 1024 * 1024 * 100);
            logLog.Start();
            for (UInt64 i = 0; !terminationEventSignal.IsCancellationRequested; i++)
            {
                bool unitsClosed = true, anyQueueEmpty = false;
                foreach (NmeaTestUnit unit in nmeaTestUnits)
                {
                    unitsClosed &= unit.InputStreamClosed;
                    anyQueueEmpty |= unit.queue.IsEmpty;
                }
                if (unitsClosed && anyQueueEmpty)
                    break;

                if (DateTime.UtcNow.Subtract(lastScreenUpdateTime).TotalSeconds > 0.8)
                {
                    lastScreenUpdateTime = DateTime.UtcNow;
                    Console.Clear();
                    bool unitsPaused = false;
                    foreach (NmeaTestUnit unit in nmeaTestUnits)
                    {
                        unitsPaused |= unit.Paused;
                    }
                    if (devLog.Paused || datLog.Paused || errLog.Paused || locLog.Paused || logLog.Paused || (collectSkew && (skewLog.Paused || skewErrLog.Paused || senLog.Paused || senErrLog.Paused)) || unitsPaused || (ppsCardUsed && ppsCard.Paused))
                    {
                        TimeSpan rem = resumeTime.Subtract(DateTime.UtcNow);
                        Console.WriteLine("Logging is stopped, {0} minutes and {1} seconds remaining. Resume by pressing F10.\n", rem.Minutes, rem.Seconds);
                    }
                    else
                    {
                        Console.WriteLine("You can pause the logging by pressing F2.\n");
                    }
                    Console.WriteLine("Measurement in progres: {0}", lable);
                    Console.WriteLine("Number of datapoints: {0}\r\nElapsed time: {1:c}", i, DateTime.UtcNow.Subtract(startTime));
                }

                while (Console.KeyAvailable)
                {
                    switch (Console.ReadKey(true).Key)
                    {
                        case ConsoleKey.F2: //pause
                            {
                                const int minToWait = 10;
                                devLog.Pause(new TimeSpan(0, minToWait, 0));
                                datLog.Pause(new TimeSpan(0, minToWait, 0));
                                errLog.Pause(new TimeSpan(0, minToWait, 0));
                                locLog.Pause(new TimeSpan(0, minToWait, 0));
                                logLog.Pause(new TimeSpan(0, minToWait, 0));

                                foreach (NmeaTestUnit unit in nmeaTestUnits)
                                {
                                    unit.PauseLoggers(new TimeSpan(0, minToWait, 0));
                                }

                                if (ppsCardUsed)
                                {
                                    ppsCard.Pause(new TimeSpan(0, minToWait, 0));
                                }

                                if (collectSkew)
                                {
                                    skewLog.Pause(new TimeSpan(0, minToWait, 0));
                                    skewErrLog.Pause(new TimeSpan(0, minToWait, 0));
                                    senLog.Pause(new TimeSpan(0, minToWait, 0));
                                    senErrLog.Pause(new TimeSpan(0, minToWait, 0));
                                }

                                resumeTime = DateTime.UtcNow.AddMinutes(minToWait);
                                Console.WriteLine("\nLogging paused for {0} minutes.", minToWait);
                            }
                            break;

                        case ConsoleKey.F10: //resume
                            {
                                devLog.ReStart();
                                datLog.ReStart();
                                errLog.ReStart();
                                locLog.ReStart();
                                logLog.ReStart();

                                foreach (NmeaTestUnit unit in nmeaTestUnits)
                                {
                                    unit.ResumeLoggers();
                                }

                                if (ppsCardUsed)
                                {
                                    ppsCard.ReStart();
                                }

                                if (collectSkew)
                                {
                                    skewLog.ReStart();
                                    skewErrLog.ReStart();
                                    senLog.ReStart();
                                    senErrLog.ReStart();
                                }
                                Console.WriteLine("\nLogging was restarted.");
                            }
                            break;

                        default:
                            break;
                    }
                }

                NmeaBlock[] currentNmeaBlocks = new NmeaBlock[nmeaTestUnits.Length];
                DateTime currentTime = DateTime.UtcNow.AddYears(50);
                for (int j = 0; j < nmeaTestUnits.Length; j++)
                {
                    int delayCount = 0;
                    while (nmeaTestUnits[j].queue.Count == 0)
                    {
                        if (terminationEventSignal.IsCancellationRequested)
                            goto TerminationSequence;

                        Thread.Sleep(delayCount);

                        if (delayCount < 100)
                            delayCount += 10;
                    }
                    while (!nmeaTestUnits[j].queue.TryPeek(out currentNmeaBlocks[j]))
                        Thread.Yield();
                    currentTime = (currentTime > currentNmeaBlocks[j].time) ? currentNmeaBlocks[j].time : currentTime;
                }

                if (0 != currentTime.Millisecond)
                {
                    string time = currentTime.AddMilliseconds(500).ToString("yyyy/MM/dd HH:mm:ss");
                    List<string> errors = new List<string>();

                    for (int j = 0; j < nmeaTestUnits.Length; j++)
                    {
                        if (currentTime == currentNmeaBlocks[j].time)
                        {
                            while (!nmeaTestUnits[j].queue.TryDequeue(out currentNmeaBlocks[j]))
                                Thread.Yield();

                            errors.Add(stringMaker("\t", time, nmeaTestUnits[j].designation, (0 == j ? "REF," : "") + "imprecise", Convert.ToString(1 << 5 | ((j == 0) ? 1 << 4 : 0), 2).PadLeft(6, '0')));
                        }
                    }

                    try
                    {
                        errLog.LogLine(String.Join("\r\n", errors));
                    }
                    catch (Exception e)
                    {
                        if (e.Message == "writeToLogFile failed")
                        {
                            Console.WriteLine("The log files cannot be accessed.");
                            Console.ReadLine();
                        }
                        else
                            throw e;
                    }

                    i--; //no new data point got captured
                    continue;
                }

                NmeaBlock refBlock = currentNmeaBlocks[0];

                string datF, devF, locF, skewF, senF;
                datF = devF = locF = skewF = senF = currentTime.ToString("yyyy/MM/dd HH:mm:ss");
                List<string> errF = new List<string>();
                StringBuilder logF = new StringBuilder();
                StringWriter logFWriter = new StringWriter(logF);
                XmlTextWriter logFXmlWriter = new XmlTextWriter(logFWriter);
                logFXmlWriter.Formatting = Formatting.Indented;
                logFXmlWriter.IndentChar = '\t';
                logFXmlWriter.Indentation = 1;

                bool refValid;
                logFXmlWriter.WriteStartElement("dataPoint");
                {
                    logFXmlWriter.WriteStartElement("time");
                    {
                        logFXmlWriter.WriteAttributeString("zone", "UTC");

                        logFXmlWriter.WriteString(currentTime.ToString("yyyy/MM/dd HH:mm:ss"));
                    }
                    logFXmlWriter.WriteEndElement();

                    refValid = refBlock.valid && (refBlock.time == currentTime) && (refBlock.fixType == FixType.Fix3D);
                    if (!refValid)
                    {
                        string errors = "REF";
                        int code = 1 << 4;
                        if (refBlock.time != currentTime) //missing
                        {
                            errors += ", missing";
                            code += 1 << 3;
                            locF = stringMaker("\t", locF, locFileString());
                        }
                        else
                        {
                            if (!refBlock.valid) //invalid
                            {
                                errors += ", invalid";
                                code += 1 << 2;
                                locF = stringMaker("\t", locF, locFileString());
                            }
                            else //no Fix3D
                            {
                                errors += ", " + (refBlock.fixType == FixType.Fix2D ? "Fix2D" : "noFix");
                                code += 1 << (refBlock.fixType == FixType.Fix2D ? 1 : 0);
                                locF = stringMaker("\t", locF, (refBlock.fixType == FixType.Fix2D) ? locFileString(refBlock) : locFileString());
                            }
                            while (!nmeaTestUnits[0].queue.TryDequeue(out currentNmeaBlocks[0]))
                                Thread.Yield();
                            blockXmler(refBlock, collectSkew, logFXmlWriter, nmeaTestUnits[0].designation, "reference", 0, nmeaTestUnits[0].boardID, nmeaTestUnits[0].boardHeight, nmeaTestUnits[0].boardWidth);
                        }
                        errF.Add(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[0].designation, errors, Convert.ToString(code, 2).PadLeft(6, '0')));
                        datF = stringMaker("\t", datF, datFileString());
                    }
                    else
                    {
                        while (!nmeaTestUnits[0].queue.TryDequeue(out currentNmeaBlocks[0]))
                            Thread.Yield();
                        datF = stringMaker("\t", datF, datFileString(refBlock));
                        locF = stringMaker("\t", locF, locFileString(refBlock));
                        blockXmler(refBlock, collectSkew, logFXmlWriter, nmeaTestUnits[0].designation, "reference", 0, nmeaTestUnits[0].boardID, nmeaTestUnits[0].boardHeight, nmeaTestUnits[0].boardWidth);
                    }

                    if (collectSkew)
                    {
                        if ((refBlock.time == currentTime) && refBlock.skewData.status == PpsStatus.OK)
                            skewF = stringMaker("\t", skewF, skewFileString(refBlock));
                        else //skew error
                        {
                            if (refBlock.time == currentTime)
                            {
                                string errorString;
                                switch (refBlock.skewData.status)
                                {
                                    case PpsStatus.notPossible:
                                        errorString = "notPossible";
                                        skewF = stringMaker("\t", skewF, skewFileString());
                                        break;

                                    case PpsStatus.glitch:
                                        errorString = "glitch";
                                        skewF = stringMaker("\t", skewF, skewFileString());
                                        break;

                                    case PpsStatus.old:
                                        errorString = "old";
                                        skewF = stringMaker("\t", skewF, skewFileString(refBlock, true));
                                        break;

                                    case PpsStatus.ppsMissing:
                                        errorString = "ppsMissing";
                                        skewF = stringMaker("\t", skewF, skewFileString());
                                        break;

                                    case PpsStatus.refMissing:
                                        errorString = "refMissing";
                                        skewF = stringMaker("\t", skewF, skewFileString(refBlock, true));
                                        break;

                                    case PpsStatus.unexpected:
                                        errorString = "unexpected";
                                        skewF = stringMaker("\t", skewF, skewFileString());
                                        break;

                                    default:
                                        errorString = "default???" + refBlock.skewData.status.ToString();
                                        skewF = stringMaker("\t", skewF, skewFileString());
                                        break;
                                }
                                skewErrLog.LogLine(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[0].designation, errorString));
                            }
                            else
                            {
                                skewF = stringMaker("\t", skewF, skewFileString());
                            }
                        }

                        if ((refBlock.time == currentTime) && refBlock.SensorData.status == SenDataStatus.OK)
                            senF = stringMaker("\t", senF, senFileString(refBlock));
                        else //sen error
                        {
                            senF = stringMaker("\t", senF, senFileString());

                            if (refBlock.time == currentTime)
                            {
                                string errorString;
                                switch (refBlock.SensorData.status)
                                {
                                    case SenDataStatus.old:
                                        errorString = "old";
                                        break;
                                    case SenDataStatus.invalid:
                                        errorString = "invalid";
                                        break;
                                    default:
                                        errorString = "default???" + refBlock.SensorData.status.ToString();
                                        break;
                                }
                                senErrLog.LogLine(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[0].designation, errorString));
                            }
                        }
                    }

                    for (int j = 1; j < nmeaTestUnits.Length; j++)
                    {
                        NmeaBlock block = currentNmeaBlocks[j];
                        if (!block.valid || (block.time != currentTime) || (block.fixType != FixType.Fix3D)) //if the block is invalid
                        {
                            string errorStr;
                            int code;
                            if (block.time != currentTime) //missing
                            {
                                errorStr = "missing";
                                code = 1 << 3;
                                locF = stringMaker("\t", locF, locFileString());
                            }
                            else
                            {
                                if (!block.valid) //invalid
                                {
                                    errorStr = "invalid";
                                    code = 1 << 2;
                                    locF = stringMaker("\t", locF, locFileString());
                                }
                                else //no Fix3D
                                {
                                    errorStr = (block.fixType == FixType.Fix2D ? "Fix2D" : "noFix");
                                    code = 1 << (block.fixType == FixType.Fix2D ? 1 : 0);
                                    locF = stringMaker("\t", locF, (block.fixType == FixType.Fix2D) ? locFileString(block) : locFileString());
                                }
                                while (!nmeaTestUnits[j].queue.TryDequeue(out currentNmeaBlocks[j]))
                                    Thread.Yield();

                                blockXmler(block, collectSkew, logFXmlWriter, nmeaTestUnits[j].designation, "DUT", j, nmeaTestUnits[j].boardID, nmeaTestUnits[j].boardHeight, nmeaTestUnits[j].boardWidth);
                            }
                            errF.Add(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[j].designation, errorStr, Convert.ToString(code, 2).PadLeft(6, '0')));
                            datF = stringMaker("\t", datF, datFileString());
                            devF = stringMaker("\t", devF, devFileString());
                        }
                        else
                        {
                            while (!nmeaTestUnits[j].queue.TryDequeue(out currentNmeaBlocks[j]))
                                Thread.Yield();
                            datF = stringMaker("\t", datF, datFileString(block));
                            locF = stringMaker("\t", locF, locFileString(block));
                            devF = stringMaker("\t", devF, refValid ? devFileString(refBlock, currentNmeaBlocks[j]) : devFileString());
                            blockXmler(block, collectSkew, logFXmlWriter, nmeaTestUnits[j].designation, "DUT", j, nmeaTestUnits[j].boardID, nmeaTestUnits[j].boardHeight, nmeaTestUnits[j].boardWidth);
                        }

                        if (collectSkew)
                        {
                            if ((block.time == currentTime) && block.skewData.status == PpsStatus.OK)
                                skewF = stringMaker("\t", skewF, skewFileString(block));
                            else //skew error
                            {
                                if (block.time == currentTime)
                                {
                                    string errorString;
                                    switch (block.skewData.status)
                                    {
                                        case PpsStatus.notPossible:
                                            errorString = "notPossible";
                                            skewF = stringMaker("\t", skewF, skewFileString());
                                            break;

                                        case PpsStatus.glitch:
                                            errorString = "glitch";
                                            skewF = stringMaker("\t", skewF, skewFileString());
                                            break;

                                        case PpsStatus.old:
                                            errorString = "old";
                                            skewF = stringMaker("\t", skewF, skewFileString(block, true));
                                            break;

                                        case PpsStatus.ppsMissing:
                                            errorString = "ppsMissing";
                                            skewF = stringMaker("\t", skewF, skewFileString());
                                            break;

                                        case PpsStatus.refMissing:
                                            errorString = "refMissing";
                                            skewF = stringMaker("\t", skewF, skewFileString(block, true));
                                            break;

                                        case PpsStatus.unexpected:
                                            errorString = "unexpected";
                                            skewF = stringMaker("\t", skewF, skewFileString());
                                            break;

                                        default:
                                            errorString = "default???" + block.skewData.status.ToString();
                                            skewF = stringMaker("\t", skewF, skewFileString());
                                            break;
                                    }
                                    skewErrLog.LogLine(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[j].designation, errorString));
                                }
                                else
                                {
                                    skewF = stringMaker("\t", skewF, skewFileString());
                                }
                            }

                            if ((block.time == currentTime) && block.SensorData.status == SenDataStatus.OK)
                                senF = stringMaker("\t", senF, senFileString(block));
                            else //sen error
                            {
                                senF = stringMaker("\t", senF, senFileString());

                                if (block.time == currentTime)
                                {
                                    string errorString;
                                    switch (block.SensorData.status)
                                    {
                                        case SenDataStatus.old:
                                            errorString = "old";
                                            break;
                                        case SenDataStatus.invalid:
                                            errorString = "invalid";
                                            break;
                                        default:
                                            errorString = "default???" + block.SensorData.status.ToString();
                                            break;
                                    }
                                    senErrLog.LogLine(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[j].designation, errorString));
                                }
                            }
                        }

                    }
                }
                logFXmlWriter.WriteEndElement();
                logFXmlWriter.Flush();
                logFWriter.WriteLine();
                logFWriter.Flush();
                try
                {
                    if (refValid)
                        devLog.LogLine(devF);
                    if (datF.Split("\t".ToCharArray(), StringSplitOptions.RemoveEmptyEntries).Length > 1)
                        datLog.LogLine(datF);
                    if (locF.Split("\t".ToCharArray(), StringSplitOptions.RemoveEmptyEntries).Length > 1)
                        locLog.LogLine(locF);
                    if (errF.Count > 0)
                        errLog.LogLine(String.Join("\r\n", errF));
                    logLog.Log(logF.ToString());

                    if (collectSkew)
                    {
                        if (skewF.Split("\t".ToCharArray(), StringSplitOptions.RemoveEmptyEntries).Length > 1)
                            skewLog.LogLine(skewF);
                        if (senF.Split("\t".ToCharArray(), StringSplitOptions.RemoveEmptyEntries).Length > 1)
                            senLog.LogLine(senF);
                    }
                }
                catch (Exception e)
                {
                    if (e.Message == "writeToLogFile failed")
                    {
                        Console.WriteLine("The log files cannot be accessed.");
                        Console.ReadLine();
                    }
                    else
                        throw e;
                }
            }
            TerminationSequence:
            foreach (var unit in nmeaTestUnits)
            {
                unit.Close();
            }
            devLog.Close();
            datLog.Close();
            locLog.Close();
            logLog.Close();
            errLog.Close();

            if (ppsCardUsed)
            {
                ppsCard.Close();
            }

            if (collectSkew)
            {
                skewLog.Close();
                skewErrLog.Close();
                senLog.Close();
                senErrLog.Close();
            }
            #endregion
        }

        static string datFileString(NmeaBlock nmeaBlock)
        {
            StringBuilder builder = new StringBuilder();
            builder.AppendFormat("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}",
                                        nmeaBlock.PDOP.ToString("F4"),
                                        nmeaBlock.HDOP.ToString("F4"),
                                        nmeaBlock.VDOP.ToString("F4"),
                                        nmeaBlock.numberOfSatellitesInView.ToString(),
                                        nmeaBlock.numberOfSatellitesInViewGPS.ToString(),
                                        nmeaBlock.numberOfSatellitesInViewGLONASS.ToString(),
                                        nmeaBlock.numberOfTrackedSatellites.ToString(),
                                        nmeaBlock.numberOfUsedSatellitesGPS.ToString(),
                                        nmeaBlock.numberOfUsedSatellitesGLONASS.ToString(),
                                        nmeaBlock.maxSNR.ToString("F4"),
                                        nmeaBlock.maxSnrGPS.ToString("F4"),
                                        nmeaBlock.maxSnrGLONASS.ToString("F4"),
                                        nmeaBlock.avrSnr.ToString("F4"),
                                        nmeaBlock.avrSnrGPS.ToString("F4"),
                                        nmeaBlock.avrSnrGLONASS.ToString("F4")
                                        );

            return builder.ToString();
        }
        static string datFileString()
        {
            return "\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
        }

        static string locFileString(NmeaBlock nmeaBlock)
        {
            StringBuilder builder = new StringBuilder();
            builder.AppendFormat("{0}\t{1}\t{2}",
                                        nmeaBlock.coordinates.Latitude,
                                        nmeaBlock.coordinates.Longitude,
                                        nmeaBlock.coordinates.Altitude
                                        );

            return builder.ToString();
        }
        static string locFileString()
        {
            return "\t\t";
        }

        static string devFileString(NmeaBlock refNmeaBlock, NmeaBlock dutNmeaBlock)
        {
            Deviance dev = new Deviance(refNmeaBlock, dutNmeaBlock);

            StringBuilder builder = new StringBuilder();
            builder.AppendFormat("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}",
                dev.offset,
                dev.PDOP,
                dev.HDOP,
                dev.VDOP,
                dev.satelliteInView,
                dev.satelliteInViewGPS,
                dev.satelliteInViewGLONASS,
                dev.satelliteUsed,
                dev.satelliteUsedGPS,
                dev.satelliteUsedGLONASS,
                dev.maxSnr,
                dev.maxSnrGPS,
                dev.maxSnrGLONASS,
                dev.avrSnr,
                dev.avrSnrGPS,
                dev.avrSnrGLONASS,
                dev.SnrDevView,
                dev.SnrDevUsed
                );

            return builder.ToString();
        }
        static string devFileString()
        {
            return "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
        }

        static string skewFileString(NmeaBlock nmeaBlock)
        {
            StringBuilder builder = new StringBuilder();
            builder.AppendFormat("{0}\t{1}",
                                        nmeaBlock.skewData.skew_ns,
                                        nmeaBlock.skewData.absTime_ticks
                                        );

            return builder.ToString();
        }
        static string skewFileString(NmeaBlock nmeaBlock, bool noSkew)
        {
            if (noSkew)
            {
                StringBuilder builder = new StringBuilder();
                builder.AppendFormat("\t{0}",
                                            nmeaBlock.skewData.absTime_ticks
                                            );

                return builder.ToString();
            }
            else
            {
                return skewFileString(nmeaBlock);
            }
        }
        static string skewFileString()
        {
            return "\t";
        }

        static string senFileString(NmeaBlock nmeaBlock)
        {
            StringBuilder builder = new StringBuilder();
            builder.AppendFormat("{0}\t{1}\t{2}\t{3}\t{4}\t{5}",
                                        nmeaBlock.SensorData.temperature_bmp180_C,
                                        nmeaBlock.SensorData.pressure_Pa,
                                        nmeaBlock.SensorData.visibleLightIntensity_lx,
                                        nmeaBlock.SensorData.infraredLighIntensity,
                                        nmeaBlock.SensorData.temperature_sht21_C,
                                        nmeaBlock.SensorData.relativeHumidity_percentage
                                        );

            return builder.ToString();
        }
        static string senFileString()
        {
            return "\t\t\t\t\t";
        }


        static string stringMaker(string separator, params string[] strings)
        {
            return String.Join(separator, strings);
        }

        static void blockXmler(NmeaBlock block, bool skewAvailable, XmlTextWriter writer, string deviceDesignation, string deviceRole, int deviceID, string boardID, double boardHeigth, double boardWidth)
        {
            writer.WriteStartElement("receiver");
            {
                writer.WriteAttributeString("designation", deviceDesignation);
                writer.WriteAttributeString("number", deviceID.ToString());
                writer.WriteAttributeString("role", deviceRole);

                writer.WriteStartElement("coordinates");
                {
                    writer.WriteStartElement("longitude");
                    {
                        writer.WriteString(block.coordinates.Longitude.ToString());
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("latitude");
                    {
                        writer.WriteString(block.coordinates.Latitude.ToString());
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("altitude");
                    {
                        writer.WriteString(block.coordinates.Altitude.ToString());
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("course");
                    {
                        writer.WriteString(block.coordinates.Course.ToString());
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("speed");
                    {
                        writer.WriteString(block.coordinates.Speed.ToString());
                    }
                    writer.WriteEndElement();
                }
                writer.WriteEndElement();

                writer.WriteStartElement("fixQuality");
                {
                    writer.WriteStartElement("type");
                    {
                        writer.WriteAttributeString("value", ((int)block.fixQuality).ToString());
                        switch (block.fixQuality)
                        {
                            case FixQuality.invalid:
                                writer.WriteString("invalid");
                                break;
                            case FixQuality.fixGPS:
                                writer.WriteString("GPS");
                                break;
                            case FixQuality.fixDGPS:
                                writer.WriteString("DGPS");
                                break;
                            case FixQuality.fixPPS:
                                writer.WriteString("PPS");
                                break;
                            case FixQuality.Real_Time_Kinematic:
                                writer.WriteString("Real Time Kinematic");
                                break;
                            case FixQuality.Float_RTK:
                                writer.WriteString("Float RTK");
                                break;
                            case FixQuality.estimated_dead_reckoning:
                                writer.WriteString("estimated dead reckoning");
                                break;
                            case FixQuality.Manual_input_mode:
                                writer.WriteString("Manual input mode");
                                break;
                            case FixQuality.Simulation_mode:
                                writer.WriteString("Simulation mode");
                                break;
                            default:
                                writer.WriteString("unkown mode");
                                break;
                        }
                    }
                    writer.WriteEndElement();

                    if (block.fixQuality == FixQuality.fixDGPS)
                    {
                        writer.WriteStartElement("DGPS_StationId");
                        {
                            if (block.DGPS_ID != -1)
                                writer.WriteString(block.DGPS_ID.ToString());
                            writer.WriteString("N/D");
                        }
                        writer.WriteEndElement();


                        writer.WriteStartElement("DGPS_Age");
                        {
                            if (block.DGPS_age != TimeSpan.MaxValue)
                                writer.WriteString(block.DGPS_age.ToString());
                            writer.WriteString("N/D");
                        }
                        writer.WriteEndElement();
                    }

                }
                writer.WriteEndElement();

                writer.WriteStartElement("numberOfTrackedSatellites");
                {
                    writer.WriteString(block.numberOfTrackedSatellites.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("maxSNR");
                {
                    writer.WriteStartElement("all");
                    {
                        writer.WriteString(block.maxSNR.ToString("F4"));
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("GPS");
                    {
                        writer.WriteString(block.maxSnrGPS.ToString("F4"));
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("GLONASS");
                    {
                        writer.WriteString(block.maxSnrGLONASS.ToString("F4"));
                    }
                    writer.WriteEndElement();
                }
                writer.WriteEndElement();

                writer.WriteStartElement("averageSNR");
                {
                    writer.WriteStartElement("all");
                    {
                        writer.WriteString(block.avrSnr.ToString("F4"));
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("GPS");
                    {
                        writer.WriteString(block.avrSnrGPS.ToString("F4"));
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("GLONASS");
                    {
                        writer.WriteString(block.avrSnrGLONASS.ToString("F4"));
                    }
                    writer.WriteEndElement();
                }
                writer.WriteEndElement();

                writer.WriteStartElement("numberOfSatellitesInView");
                {
                    writer.WriteString(block.numberOfSatellitesInView.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("fixType");
                {
                    writer.WriteAttributeString("value", ((int)block.fixType).ToString());
                    switch (block.fixType)
                    {
                        case FixType.noFix:
                            writer.WriteString("no fix");
                            break;
                        case FixType.Fix2D:
                            writer.WriteString("2D");
                            break;
                        case FixType.Fix3D:
                            writer.WriteString("3D");
                            break;
                        default:
                            writer.WriteString("unkown type");
                            break;
                    }
                }
                writer.WriteEndElement();

                writer.WriteStartElement("PDOP");
                {
                    writer.WriteString(block.PDOP.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("HDOP");
                {
                    writer.WriteString(block.HDOP.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("VDOP");
                {
                    writer.WriteString(block.VDOP.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("usedSatellitesGPS");
                {
                    writer.WriteAttributeString("number", block.numberOfUsedSatellitesGPS.ToString());
                    foreach (int PRN in block.usedSatellitesGPS)
                    {
                        writer.WriteStartElement("PRN");
                        {
                            writer.WriteString(PRN.ToString());
                        }
                        writer.WriteEndElement();
                    }
                }
                writer.WriteEndElement();

                writer.WriteStartElement("satellitesGPS");
                {
                    writer.WriteAttributeString("number", block.numberOfSatellitesInViewGPS.ToString());
                    foreach (SatelliteData satellite in block.satellitesGPS)
                    {
                        satelliteDateXmler(satellite, writer, "GPS");
                    }
                }
                writer.WriteEndElement();

                writer.WriteStartElement("usedSatellitesGLONASS");
                {
                    writer.WriteAttributeString("number", block.numberOfUsedSatellitesGLONASS.ToString());
                    foreach (int PRN in block.usedSatellitesGLONASS)
                    {
                        writer.WriteStartElement("PRN");
                        {
                            writer.WriteString(PRN.ToString());
                        }
                        writer.WriteEndElement();
                    }
                }
                writer.WriteEndElement();

                writer.WriteStartElement("satellitesGLONASS");
                {
                    writer.WriteAttributeString("number", block.numberOfSatellitesInViewGLONASS.ToString());
                    foreach (SatelliteData satellite in block.satellitesGLONASS)
                    {
                        satelliteDateXmler(satellite, writer, "GLONASS");
                    }
                }
                writer.WriteEndElement();

                if (skewAvailable)
                {
                    writer.WriteStartElement("skew");
                    {
                        switch (block.skewData.status)
                        {
                            case PpsStatus.OK:
                                writer.WriteAttributeString("status", "OK");
                                break;

                            case PpsStatus.old:
                                writer.WriteAttributeString("status", "old");
                                break;

                            case PpsStatus.glitch:
                                writer.WriteAttributeString("status", "glitch");
                                break;

                            case PpsStatus.ppsMissing:
                                writer.WriteAttributeString("status", "ppsMissing");
                                break;

                            case PpsStatus.refMissing:
                                writer.WriteAttributeString("status", "refMissing");
                                break;

                            case PpsStatus.notPossible:
                                writer.WriteAttributeString("status", "notPossible");
                                break;

                            case PpsStatus.unexpected:
                                writer.WriteAttributeString("status", "unexpected");
                                break;
                        }

                        writer.WriteStartElement("skew");
                        {
                            writer.WriteAttributeString("unit", "ns");
                            writer.WriteString(block.skewData.skew_ns.ToString());
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("absoluteTime");
                        {
                            writer.WriteAttributeString("unit", "ticks");
                            writer.WriteString(block.skewData.absTime_ticks.ToString());
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("packetDelay");
                        {
                            writer.WriteAttributeString("unit", "ms");
                            writer.WriteString(block.skewData.packetDelay_ms.ToString());
                        }
                        writer.WriteEndElement();
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("sensorData");
                    {
                        switch (block.SensorData.status)
                        {
                            case SenDataStatus.OK:
                                writer.WriteAttributeString("status", "OK");
                                break;

                            case SenDataStatus.old:
                                writer.WriteAttributeString("status", "old");
                                break;

                            case SenDataStatus.invalid:
                                writer.WriteAttributeString("status", "invalid");
                                break;
                        }

                        writer.WriteStartElement("temperature");
                        {
                            writer.WriteStartElement("bmp180");
                            {
                                writer.WriteAttributeString("unit", "C");
                                writer.WriteString(block.SensorData.temperature_bmp180_C.ToString());
                            }
                            writer.WriteEndElement();

                            writer.WriteStartElement("sht21");
                            {
                                writer.WriteAttributeString("unit", "C");
                                writer.WriteString(block.SensorData.temperature_sht21_C.ToString());
                            }
                            writer.WriteEndElement();
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("pressure");
                        {
                            writer.WriteAttributeString("unit", "Pa");
                            writer.WriteString(block.SensorData.pressure_Pa.ToString());
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("visibleLightIntensity");
                        {
                            writer.WriteAttributeString("unit", "lx");
                            writer.WriteString(block.SensorData.visibleLightIntensity_lx.ToString());
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("infraredLightIntensity");
                        {
                            writer.WriteAttributeString("unit", "unknown");
                            writer.WriteString(block.SensorData.infraredLighIntensity.ToString());
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("relativeHumidity");
                        {
                            writer.WriteAttributeString("unit", "%");
                            writer.WriteString(block.SensorData.relativeHumidity_percentage.ToString());
                        }
                        writer.WriteEndElement();

                        writer.WriteStartElement("sampleAge");
                        {
                            writer.WriteAttributeString("unit", "ms");
                            writer.WriteString(block.SensorData.sampleAge_ms.ToString());
                        }
                        writer.WriteEndElement();
                    }
                    writer.WriteEndElement();
                }

                writer.WriteStartElement("raw");
                {
                    foreach (string message in block.raw)
                    {
                        writer.WriteStartElement("message");
                        {
                            writer.WriteString(message);
                        }
                        writer.WriteEndElement();
                    }
                }
                writer.WriteEndElement();

                writer.WriteStartElement("board");
                {
                    writer.WriteStartElement("ID");
                    {
                        writer.WriteString(boardID);
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("height");
                    {
                        writer.WriteAttributeString("unit", "mm");
                        writer.WriteString(boardHeigth.ToString());
                    }
                    writer.WriteEndElement();

                    writer.WriteStartElement("width");
                    {
                        writer.WriteAttributeString("unit", "mm");
                        writer.WriteString(boardWidth.ToString());
                    }
                    writer.WriteEndElement();
                }
                writer.WriteEndElement();
            }
            writer.WriteEndElement();
        }
        static void blockXmler_simple(NmeaBlock block, XmlTextWriter writer, string deviceDesignation, string deviceRole, int deviceID, string boardID, double boardHeigth, double boardWidth)
        {
            blockXmler(block, false, writer, deviceDesignation, deviceRole, deviceID, boardID, boardHeigth, boardWidth);
        }

        static void satelliteDateXmler(SatelliteData satelliteData, XmlTextWriter writer, string GNSS)
        {
            writer.WriteStartElement("satellite");
            {
                writer.WriteAttributeString("PRN", satelliteData.PRN.ToString());
                writer.WriteAttributeString("GNSS", GNSS);
                writer.WriteStartElement("SNR");
                {
                    writer.WriteString(satelliteData.SNR.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("azimuth");
                {
                    writer.WriteString(satelliteData.azimuth.ToString());
                }
                writer.WriteEndElement();

                writer.WriteStartElement("elevation");
                {
                    writer.WriteString(satelliteData.elevation.ToString());
                }
                writer.WriteEndElement();
            }
            writer.WriteEndElement();
        }

        class Deviance
        {
            public readonly double offset;

            public readonly double PDOP;
            public readonly double HDOP;
            public readonly double VDOP;

            public readonly double satelliteInView;
            public readonly double satelliteUsed;
            public readonly double satelliteInViewGPS;
            public readonly double satelliteUsedGPS;
            public readonly double satelliteInViewGLONASS;
            public readonly double satelliteUsedGLONASS;

            public readonly double maxSnr;
            public readonly double maxSnrGPS;
            public readonly double maxSnrGLONASS;
            public readonly double avrSnr;
            public readonly double avrSnrGPS;
            public readonly double avrSnrGLONASS;

            public readonly double SnrDevUsed;
            public readonly double SnrDevView;

            public Deviance(NmeaBlock reference, NmeaBlock DUT)
            {
                if (!reference.valid || !DUT.valid)
                    throw new Exception("Deviance: invalid");

                offset = Math.Sqrt(Math.Pow(DUT.coordinates.GetDistanceTo(reference.coordinates), 2) + Math.Pow((reference.coordinates.Altitude - DUT.coordinates.Altitude), 2));

                PDOP = relativeError(reference.PDOP, DUT.PDOP);
                HDOP = relativeError(reference.HDOP, DUT.HDOP);
                VDOP = relativeError(reference.VDOP, DUT.VDOP);

                satelliteInView = relativeError(reference.numberOfSatellitesInView, DUT.numberOfSatellitesInView);
                satelliteUsed = relativeError(reference.numberOfTrackedSatellites, DUT.numberOfTrackedSatellites);
                satelliteInViewGPS = relativeError(reference.numberOfSatellitesInViewGPS, DUT.numberOfSatellitesInViewGPS);
                satelliteUsedGPS = relativeError(reference.numberOfUsedSatellitesGPS, DUT.numberOfUsedSatellitesGPS);
                satelliteInViewGLONASS = relativeError(reference.numberOfSatellitesInViewGLONASS, DUT.numberOfSatellitesInViewGLONASS);
                satelliteUsedGLONASS = relativeError(reference.numberOfUsedSatellitesGLONASS, DUT.numberOfUsedSatellitesGLONASS);

                maxSnr = DUT.maxSNR - reference.maxSNR;
                maxSnrGPS = DUT.maxSnrGPS - reference.maxSnrGPS;
                maxSnrGLONASS = DUT.maxSnrGLONASS - reference.maxSnrGLONASS;
                avrSnr = DUT.avrSnr - reference.avrSnr;
                avrSnrGPS = DUT.avrSnrGPS - reference.avrSnrGPS;
                avrSnrGLONASS = DUT.avrSnrGLONASS - reference.avrSnrGLONASS;

                IEnumerable<int> PRNs = reference.usedSatellites.Union(DUT.usedSatellites);
                SnrDevUsed = 0;
                foreach (int PRN in PRNs)
                {
                    double SNR_ref = reference.satellites.Where((x) => !double.IsNaN(x.SNR)).Select((x) => x.PRN).Contains(PRN) ? reference.satellites.First(x => x.PRN == PRN).SNR : 0;
                    double SNR_DUT = DUT.satellites.Where((x) => !double.IsNaN(x.SNR)).Select((x) => x.PRN).Contains(PRN) ? DUT.satellites.First(x => x.PRN == PRN).SNR : 0;
                    SnrDevUsed += SNR_DUT - SNR_ref;
                }
                SnrDevUsed /= PRNs.Count();

                PRNs = reference.satellites.Where((x) => !double.IsNaN(x.SNR)).Select((x) => x.PRN).Union(DUT.satellites.Where((x) => !double.IsNaN(x.SNR)).Select((x) => x.PRN));
                SnrDevView = 0;
                foreach (int PRN in PRNs)
                {
                    double SNR_ref = reference.satellites.Where((x) => !double.IsNaN(x.SNR)).Select((x) => x.PRN).Contains(PRN) ? reference.satellites.First(x => x.PRN == PRN).SNR : 0;
                    double SNR_DUT = DUT.satellites.Where((x) => !double.IsNaN(x.SNR)).Select((x) => x.PRN).Contains(PRN) ? DUT.satellites.First(x => x.PRN == PRN).SNR : 0;
                    SnrDevView += SNR_DUT - SNR_ref;
                }
                SnrDevView /= PRNs.Count();
            }
            private double relativeError(double x, double aprx)
            {
                if (x == aprx)
                    return 0;

                if (x != 0)
                    return Math.Sign(aprx - x) * Math.Abs((aprx - x) / x);
                else return 1;
            }
        }

        #region Trap application termination

        static CancellationTokenSource terminationEventSignal = new CancellationTokenSource();

        [DllImport("Kernel32")]
        private static extern bool SetConsoleCtrlHandler(TerminationEventHandler handler, bool add);

        private delegate bool TerminationEventHandler(CtrlType sig);
        static TerminationEventHandler terminationEvent = new TerminationEventHandler((type) => terminationEventHandler(type));

        enum CtrlType
        {
            CTRL_C_EVENT = 0,
            CTRL_BREAK_EVENT = 1,
            CTRL_CLOSE_EVENT = 2,
            CTRL_LOGOFF_EVENT = 5,
            CTRL_SHUTDOWN_EVENT = 6
        }

        private static bool terminationEventHandler(CtrlType sig)
        {
            terminationEventSignal.Cancel();
            if (running)
                SpinWait.SpinUntil(() => false);
            return true;
        }
        #endregion

        private static string GetMD5()
        {
            System.Security.Cryptography.MD5CryptoServiceProvider md5 = new System.Security.Cryptography.MD5CryptoServiceProvider();
            System.IO.FileStream stream = new System.IO.FileStream(System.Diagnostics.Process.GetCurrentProcess().MainModule.FileName, System.IO.FileMode.Open, System.IO.FileAccess.Read);

            md5.ComputeHash(stream);

            stream.Close();

            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            for (int i = 0; i < md5.Hash.Length; i++)
                sb.Append(md5.Hash[i].ToString("x2"));

            return sb.ToString().ToUpperInvariant();
        }
    }
}
