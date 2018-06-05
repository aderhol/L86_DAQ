using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NmeaParser;
using NmeaParser.Nmea;
using NmeaParser.Nmea.Gnss;
using NmeaParser.Nmea.Gps;
using NmeaParser.Nmea.Glonass;
using System.IO;
using System.IO.Ports;
using System.Device.Location;
using System.Collections.Concurrent;
using System.Threading;
using System.Xml;
using System.Runtime.InteropServices;

namespace L86_collector
{
    class Program
    {
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
        private class NmeaBlock
        {
            public DateTime time;
            public bool valid;
            public GeoCoordinate coordinates;
            public FixQuality fixQuality;
            public TimeSpan DGPS_age;
            public int DGPS_ID;
            public int numberOfTrackedSatellites;
            public int numberOfSatellitesInView { get { return numberOfSatellitesInViewGPS + numberOfSatellitesInViewGLONASS; } }
            public FixType fixType;
            public double PDOP;
            public double HDOP;
            public double VDOP;

            public int[] usedSatellitesGPS;
            public int numberOfUsedSatellitesGPS { get { return usedSatellitesGPS.Length; } }
            public SatelliteData[] satellitesGPS;
            public int numberOfSatellitesInViewGPS;
            public double maxSnrGPS { get { if (satellitesGPS.Length == 0) return double.NaN; else return satellitesGPS.Select(x => x.SNR).Max(); } }
            public double avrSnrGPS { get { if (satellitesGPS.Length == 0) return double.NaN; else return satellitesGPS.Select(x => x.SNR).Average(); } }

            public int[] usedSatellitesGLONASS;
            public int numberOfUsedSatellitesGLONASS { get { return usedSatellitesGLONASS.Length; } }
            public SatelliteData[] satellitesGLONASS;
            public int numberOfSatellitesInViewGLONASS;
            public double maxSnrGLONASS { get { if (satellitesGLONASS.Length == 0) return double.NaN; else return satellitesGLONASS.Select(x => x.SNR).Max(); } }
            public double avrSnrGLONASS { get { if (satellitesGLONASS.Length == 0) return double.NaN; else return satellitesGLONASS.Select(x => x.SNR).Average(); } }

            public double maxSNR { get { return Math.Max(maxSnrGPS, maxSnrGLONASS); } }
            public double avrSnr { get { if (satellites.Length == 0) return double.NaN; else return satellites.Select(x => x.SNR).Average(); } }

            public int[] usedSatellites
            {
                get
                {
                    int[] all = new int[numberOfTrackedSatellites];
                    usedSatellitesGPS.CopyTo(all, 0);
                    usedSatellitesGLONASS.CopyTo(all, usedSatellitesGPS.Length);
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
                    return all;
                }
            }

            public List<string> raw = new List<string>();
        }

        class NmeaDeviceReader
        {
            private bool init = false;
            private NmeaBlock nmeaBlock = null;
            private List<SatelliteData> satellitesGPS = null;
            private List<SatelliteData> satellitesGLONASS = null;
            private int GSAindex = 0;
            private readonly ConcurrentQueue<NmeaBlock> queue;
            private readonly SerialPortDevice port;
            private readonly string rawFile;

            public NmeaDeviceReader(string portNum, ConcurrentQueue<NmeaBlock> queue_, string rawFile)
            {
                queue = queue_;
                port = new SerialPortDevice(new SerialPort("COM" + portNum, 9600, Parity.None, 8, StopBits.One));
                port.MessageReceived += NmeaMessageReceived;
                port.OpenAsync();
                this.rawFile = rawFile;
            }
            private void NmeaMessageReceived(object sender_, EventArgs args_)
            {

                NmeaDevice device = (NmeaDevice)sender_;
                NmeaMessage message_ = ((NmeaMessageReceivedEventArgs)args_).Message;
                if (!init)
                {
                    if (message_.MessageType == "GPTXT")
                    {
                        init = true;
                        satellitesGPS = new List<SatelliteData>();
                        satellitesGLONASS = new List<SatelliteData>();
                        GSAindex = 0;
                        nmeaBlock = new NmeaBlock();
                    }
                    return;
                }
                using (StreamWriter write = new StreamWriter(rawFile, true))
                    write.WriteLine(message_.ToString());
                nmeaBlock.raw.Add(message_.ToString());
                switch (message_.MessageType)
                {
                    case "GPRMC":
                        {
                            Gprmc message = (Gprmc)message_;
                            nmeaBlock.time = message.FixTime;
                            nmeaBlock.valid = message.Active;
                            nmeaBlock.coordinates = new GeoCoordinate(message.Latitude, message.Longitude);
                            nmeaBlock.coordinates.Speed = message.Speed * 0.51444444444;
                            nmeaBlock.coordinates.Course = message.Course;
                        }
                        break;
                    case "GNRMC":
                        {
                            Gnrmc message = (Gnrmc)message_;
                            nmeaBlock.time = message.FixTime;
                            nmeaBlock.valid = message.Active;
                            nmeaBlock.coordinates = new GeoCoordinate(message.Latitude, message.Longitude);
                            nmeaBlock.coordinates.Speed = message.Speed * 0.51444444444;
                            nmeaBlock.coordinates.Course = message.Course;
                        }
                        break;
                    case "GPGGA":
                        {
                            Gpgga message = (Gpgga)message_;
                            nmeaBlock.fixQuality = (FixQuality)message.Quality;
                            nmeaBlock.numberOfTrackedSatellites = message.NumberOfSatellites;
                            nmeaBlock.coordinates.Altitude = message.Altitude;
                            nmeaBlock.DGPS_age = message.TimeSinceLastDgpsUpdate;
                            nmeaBlock.DGPS_ID = message.DgpsStationId;
                        }
                        break;
                    case "GNGGA":
                        {
                            Gngga message = (Gngga)message_;
                            nmeaBlock.fixQuality = (FixQuality)message.Quality;
                            nmeaBlock.numberOfTrackedSatellites = message.NumberOfSatellites;
                            nmeaBlock.coordinates.Altitude = message.Altitude;
                            nmeaBlock.DGPS_age = message.TimeSinceLastDgpsUpdate;
                            nmeaBlock.DGPS_ID = message.DgpsStationId;
                        }
                        break;
                    case "GPGSA":
                        {
                            Gpgsa message = (Gpgsa)message_;
                            nmeaBlock.fixType = (FixType)message.FixMode;
                            nmeaBlock.HDOP = message.Hdop;
                            nmeaBlock.VDOP = message.Vdop;
                            nmeaBlock.PDOP = message.Pdop;
                            if (GSAindex++ == 0)
                                nmeaBlock.usedSatellitesGPS = message.SVs.ToArray();
                            else
                                nmeaBlock.usedSatellitesGLONASS = message.SVs.ToArray();
                        }
                        break;
                    case "GNGSA":
                        {
                            Gngsa message = (Gngsa)message_;
                            nmeaBlock.fixType = (FixType)message.FixMode;
                            nmeaBlock.HDOP = message.Hdop;
                            nmeaBlock.VDOP = message.Vdop;
                            nmeaBlock.PDOP = message.Pdop;
                            if (GSAindex++ == 0)
                                nmeaBlock.usedSatellitesGPS = message.SVs.OrderBy((x) => x).ToArray();
                            else
                                nmeaBlock.usedSatellitesGLONASS = message.SVs.OrderBy((x) => x).ToArray();
                        }
                        break;
                    case "GPGSV":
                        {
                            Gpgsv message = (Gpgsv)message_;
                            nmeaBlock.numberOfSatellitesInViewGPS = message.SVsInView;
                            foreach (SatelliteVehicle sat in message.SVs)
                                satellitesGPS.Add(new SatelliteData(sat));
                        }
                        break;
                    case "GLGSV":
                        {
                            Glgsv message = (Glgsv)message_;
                            nmeaBlock.numberOfSatellitesInViewGLONASS = message.SVsInView;
                            foreach (SatelliteVehicle sat in message.SVs)
                                satellitesGLONASS.Add(new SatelliteData(sat));
                        }
                        break;
                    case "GPTXT":
                        nmeaBlock.satellitesGPS = satellitesGPS.OrderBy((x) => x.PRN).ToArray();
                        nmeaBlock.satellitesGLONASS = satellitesGLONASS.OrderBy((x) => x.PRN).ToArray();

                        queue.Enqueue(nmeaBlock);

                        satellitesGPS = new List<SatelliteData>();
                        satellitesGLONASS = new List<SatelliteData>();
                        nmeaBlock = new NmeaBlock();
                        GSAindex = 0;
                        break;
                    case "GNTXT":
                        nmeaBlock.satellitesGPS = satellitesGPS.ToArray();
                        nmeaBlock.satellitesGLONASS = satellitesGLONASS.ToArray();

                        queue.Enqueue(nmeaBlock);

                        satellitesGPS = new List<SatelliteData>();
                        satellitesGLONASS = new List<SatelliteData>();
                        nmeaBlock = new NmeaBlock();
                        GSAindex = 0;
                        break;
                    default:
                        break;
                }
            }
        }

        static ConcurrentQueue<NmeaBlock> queue_ref = new ConcurrentQueue<NmeaBlock>();
        static ConcurrentQueue<NmeaBlock> queue_1 = new ConcurrentQueue<NmeaBlock>();
        static void Main(string[] args)
        {
            SetConsoleCtrlHandler(terminationEventHandler, true);

            Console.Write("ref COM: ");
            string com_ref_in = Console.ReadLine();
            Console.Write("COM 1: ");
            string com_1_in = Console.ReadLine();
            Console.Write("Label of measurement: ");
            string lable = Console.ReadLine();
            Console.Write("Work directory: ");
            string folder = /*Console.ReadLine();*/ @"C:\Users\Adam\Desktop\GND Size Study\";
            string workFolder = Path.Combine(folder, lable) + @"\";

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

            try
            {
                if (!File.Exists(workFolder + lable + ".dev"))
                    using (StreamWriter write = new StreamWriter(workFolder + lable + ".dev", false))
                        write.WriteLine("time (UTC)\toffset\tPDOP\tHDOP\tVDOP\tsatelliteInView\tsatelliteUsed\tsatelliteInViewGPS\tsatelliteUsedGPS\tsatelliteInViewGLONASS\tsatelliteUsedGLONASS\tmaxSnr\tmaxSnrGPS\tmaxSnrGLONASS\tavrSnr\tavrSnrGPS\tavrSnrGLONASS\tSnrDevUsed\tSnrDevView");

                if (!File.Exists(workFolder + lable + ".dat"))
                    using (StreamWriter write = new StreamWriter(workFolder + lable + ".dat", false))
                        write.WriteLine("time (UTC)\tref PDOP\tDUT_1 PDOP\tref HDOP\tDUT_1 HDOP\tref VDOP\tDUT_1 VDOP\tref satelliteInView\tDUT_1 satelliteInView\tref satelliteUsed\tDUT_1 satelliteUsed\tref satelliteInViewGPS\tDUT_1 satelliteInViewGPS\tref satelliteUsedGPS\tDUT_1 satelliteUsedGPS\tref satelliteInViewGLONASS\tDUT_1 satelliteInViewGLONASS\tref satelliteUsedGLONASS\tDUT_1 satelliteUsedGLONASS\tref maxSnr\tDUT_1 maxSnr\tref maxSnrGPS\tDUT_1 maxSnrGPS\tref maxSnrGLONASS\tDUT_1 maxSnrGLONASS\tref avrSnr\tDUT_1 avrSnr\tref avrSnrGPS\tDUT_1 avrSnrGPS\tref avrSnrGLONASS\tDUT_1 avrSnrGLONASS");

                if (!File.Exists(workFolder + lable + ".err"))
                    using (StreamWriter write = new StreamWriter(workFolder + lable + ".err", false))
                        write.WriteLine("time (UTC)\tdevice\ttype");

                if (!File.Exists(workFolder + lable + ".loc"))
                    using (StreamWriter write = new StreamWriter(workFolder + lable + ".loc", false))
                        write.WriteLine("time (UTC)\treference latitude\treference longitude\treference altitude\tDUT_1 latitude\tDUT_1 longitude\tDUT_1 altitude");

            }
            catch (Exception)
            {
                Console.WriteLine("Bad lable!");
                Console.ReadLine();
                Environment.Exit(0);
            }

            NmeaDeviceReader reference, DUT_1;
            try
            {
                reference = new NmeaDeviceReader(com_ref_in, queue_ref, workFolder + lable + "_reference.raw");
                DUT_1 = new NmeaDeviceReader(com_1_in, queue_1, workFolder + lable + "_DUT_1.raw");
            }
            catch (Exception)
            {
                Console.WriteLine("COM port error!");
                Console.ReadLine();
                Environment.Exit(0);
            }

            running = true;

            DateTime startTime = DateTime.UtcNow;
            for (UInt64 i = 0; !terminationEventSignal.IsCancellationRequested; i++)
            {
                Console.Clear();
                Console.WriteLine("Number of datapoints: {0}\r\nElapsed time: {1:c}", i, DateTime.UtcNow.Subtract(startTime));
                NmeaBlock nmeaBlock_ref, nmeaBlock_1;
                SpinWait.SpinUntil(() => queue_ref.Count > 0);
                while (!queue_ref.TryPeek(out nmeaBlock_ref))
                    Task.Delay(1).Wait();
                SpinWait.SpinUntil(() => queue_1.Count > 0);
                while (!queue_1.TryPeek(out nmeaBlock_1))
                    Task.Delay(1).Wait();

                Deviance deviance;
                if (nmeaBlock_ref.time == nmeaBlock_1.time)
                {
                    while (!queue_ref.TryDequeue(out nmeaBlock_ref))
                        Task.Delay(1).Wait();
                    while (!queue_1.TryDequeue(out nmeaBlock_1))
                        Task.Delay(1).Wait();

                    deviance = new Deviance(nmeaBlock_ref, nmeaBlock_1);
                }
                else if (nmeaBlock_ref.time < nmeaBlock_1.time)
                {
                    while (!queue_ref.TryDequeue(out nmeaBlock_ref))
                        Task.Delay(1).Wait();

                    deviance = new Deviance(nmeaBlock_ref.time, Device.reference);
                }
                else
                {
                    while (!queue_1.TryDequeue(out nmeaBlock_1))
                        Task.Delay(1).Wait();

                    deviance = new Deviance(nmeaBlock_1.time, Device.DUT_1);
                }

                bool devWritten = false, logWritten = false, datWritten = false, locWritten = false, errWritten = false, DEB_EX;

                for (int j = 0; DEB_EX = (!devWritten || !logWritten || !datWritten || !locWritten || !errWritten); j++)
                {
                    if (j > 1000)
                    {
                        Console.WriteLine("The log files cannot be accessed!");
                        Console.ReadLine();
                    }
                    if (deviance.valid)
                    {
                        errWritten = true;
                        try
                        {
                            if (!devWritten)
                            {
                                using (StreamWriter write = new StreamWriter(workFolder + lable + ".dev", true))
                                {
                                    write.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}\t{18}", deviance.time.ToString("yyyy/MM/dd HH:mm:ss"),
                                        deviance.offset.ToString("F4"),
                                        deviance.PDOP.ToString("F4"),
                                        deviance.HDOP.ToString("F4"),
                                        deviance.VDOP.ToString("F4"),
                                        deviance.satelliteInView.ToString("F4"),
                                        deviance.satelliteUsed.ToString("F4"),
                                        deviance.satelliteInViewGPS.ToString("F4"),
                                        deviance.satelliteUsedGPS.ToString("F4"),
                                        deviance.satelliteInViewGLONASS.ToString("F4"),
                                        deviance.satelliteUsedGLONASS.ToString("F4"),
                                        deviance.maxSnr.ToString("F4"),
                                        deviance.maxSnrGPS.ToString("F4"),
                                        deviance.maxSnrGLONASS.ToString("F4"),
                                        deviance.avrSnr.ToString("F4"),
                                        deviance.avrSnrGPS.ToString("F4"),
                                        deviance.avrSnrGLONASS.ToString("F4"),
                                        deviance.SnrDevUsed.ToString("F4"),
                                        deviance.SnrDevView.ToString("F4"));
                                }
                                devWritten = true;
                            }

                            if (!datWritten)
                            {
                                using (StreamWriter write = new StreamWriter(workFolder + lable + ".dat", true))
                                {
                                    write.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\t{14}\t{15}\t{16}\t{17}\t{18}\t{19}\t{20}\t{21}\t{22}\t{23}\t{24}\t{25}\t{26}\t{27}\t{28}\t{29}\t{30}", deviance.time.ToString("yyyy/MM/dd HH:mm:ss"),
                                        nmeaBlock_ref.PDOP.ToString("F4"),
                                        nmeaBlock_1.PDOP.ToString("F4"),
                                        nmeaBlock_ref.HDOP.ToString("F4"),
                                        nmeaBlock_1.HDOP.ToString("F4"),
                                        nmeaBlock_ref.VDOP.ToString("F4"),
                                        nmeaBlock_1.VDOP.ToString("F4"),
                                        nmeaBlock_ref.numberOfSatellitesInView.ToString(),
                                        nmeaBlock_1.numberOfSatellitesInView.ToString(),
                                        nmeaBlock_ref.numberOfTrackedSatellites.ToString(),
                                        nmeaBlock_1.numberOfTrackedSatellites.ToString(),
                                        nmeaBlock_ref.numberOfSatellitesInViewGPS.ToString(),
                                        nmeaBlock_1.numberOfSatellitesInViewGPS.ToString(),
                                        nmeaBlock_ref.numberOfUsedSatellitesGPS.ToString(),
                                        nmeaBlock_1.numberOfUsedSatellitesGPS.ToString(),
                                        nmeaBlock_ref.numberOfSatellitesInViewGLONASS.ToString(),
                                        nmeaBlock_1.numberOfSatellitesInViewGLONASS.ToString(),
                                        nmeaBlock_ref.numberOfUsedSatellitesGLONASS.ToString(),
                                        nmeaBlock_1.numberOfUsedSatellitesGLONASS.ToString(),
                                        nmeaBlock_ref.maxSNR.ToString("F4"),
                                        nmeaBlock_1.maxSNR.ToString("F4"),
                                        nmeaBlock_ref.maxSnrGPS.ToString("F4"),
                                        nmeaBlock_1.maxSnrGPS.ToString("F4"),
                                        nmeaBlock_ref.maxSnrGLONASS.ToString("F4"),
                                        nmeaBlock_1.maxSnrGLONASS.ToString("F4"),
                                        nmeaBlock_ref.avrSnr.ToString("F4"),
                                        nmeaBlock_1.avrSnr.ToString("F4"),
                                        nmeaBlock_ref.avrSnrGPS.ToString("F4"),
                                        nmeaBlock_1.avrSnrGPS.ToString("F4"),
                                        nmeaBlock_ref.avrSnrGLONASS.ToString("F4"),
                                        nmeaBlock_1.avrSnrGLONASS.ToString("F4"));
                                }
                                datWritten = true;
                            }

                            if (!locWritten)
                            {
                                using (StreamWriter write = new StreamWriter(workFolder + lable + ".loc", true))
                                {
                                    write.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}", deviance.time.ToString("yyyy/MM/dd HH:mm:ss"),
                                        nmeaBlock_ref.coordinates.Latitude,
                                        nmeaBlock_ref.coordinates.Longitude,
                                        nmeaBlock_ref.coordinates.Altitude,
                                        nmeaBlock_1.coordinates.Latitude,
                                        nmeaBlock_1.coordinates.Longitude,
                                        nmeaBlock_1.coordinates.Altitude);
                                }
                                locWritten = true;
                            }
                        }
                        catch (Exception)
                        {
                            Task.Delay(10).Wait();
                        }
                    }
                    else
                    {
                        devWritten = datWritten = locWritten = true;
                        if (!errWritten)
                        {
                            try
                            {
                                using (StreamWriter write = new StreamWriter(workFolder + lable + ".err", true))
                                {
                                    string device;
                                    switch (deviance.device)
                                    {
                                        case Device.reference:
                                            device = "reference";
                                            break;
                                        case Device.DUT_1:
                                            device = "DUT_1";
                                            break;
                                        default:
                                            throw new Exception("non-existent device");
                                            break;
                                    }
                                    string type;
                                    switch (deviance.type)
                                    {
                                        case Type.invalid:
                                            type = "invalid";
                                            break;
                                        case Type.mismatch:
                                            type = "mismatch";
                                            break;
                                        default:
                                            throw new Exception("non-existent type");
                                            break;
                                    }
                                    write.WriteLine("{0}\t{1}\t{2}", deviance.time.ToString("yyyy/MM/dd HH:mm:ss"), device, type);
                                }
                                errWritten = true;
                            }
                            catch (Exception)
                            {
                                Task.Delay(10).Wait();
                            }
                        }
                    }

                    if (!logWritten)
                    {
                        try
                        {
                            using (FileStream logFile = File.Open(workFolder + lable + ".log", FileMode.Append))
                            using (StreamWriter twriter = new StreamWriter(logFile, Encoding.UTF8))
                            using (XmlTextWriter writer = new XmlTextWriter(twriter))
                            {
                                writer.Formatting = Formatting.Indented;
                                writer.IndentChar = '\t';
                                writer.Indentation = 1;

                                writer.WriteStartElement("dataPoint");
                                {
                                    writer.WriteStartElement("time");
                                    {
                                        writer.WriteAttributeString("zone", "UTC");
                                        if (!deviance.valid && deviance.type == Type.mismatch && deviance.device == Device.DUT_1)
                                            writer.WriteString(nmeaBlock_1.time.ToString("yyyy/MM/dd HH:mm:ss"));
                                        else
                                            writer.WriteString(nmeaBlock_ref.time.ToString("yyyy/MM/dd HH:mm:ss"));
                                    }
                                    writer.WriteEndElement();

                                    if (!(!deviance.valid && deviance.type == Type.mismatch && deviance.device == Device.DUT_1))
                                    {
                                        blockXmler(nmeaBlock_ref, writer, "reference", 0);
                                    }

                                    if (!(!deviance.valid && deviance.type == Type.mismatch && deviance.device == Device.reference))
                                    {
                                        blockXmler(nmeaBlock_1, writer, "DUT_1", 1);
                                    }
                                }
                                writer.WriteEndElement();

                                writer.Flush();
                                twriter.WriteLine("\r\n");
                            }
                            logWritten = true;
                        }
                        catch (Exception)
                        {
                            Task.Delay(10).Wait();
                        }
                    }
                }

            }

        }

        static void blockXmler(NmeaBlock block, XmlTextWriter writer, string deviceName, int deviceID)
        {
            writer.WriteStartElement("receiver");
            {
                writer.WriteAttributeString("designation", deviceName);
                writer.WriteAttributeString("number", deviceID.ToString());

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
            }
            writer.WriteEndElement();
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

        enum Type
        {
            invalid,
            mismatch
        }
        enum Device
        {
            reference,
            DUT_1
        }
        class Deviance
        {
            public readonly DateTime time;

            public readonly bool valid;
            public readonly Device device;
            public readonly Type type;

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

            public Deviance(DateTime time, Device device)
            {
                this.time = time;
                this.device = device;
                type = Type.mismatch;
                valid = false;
            }

            public Deviance(NmeaBlock reference, NmeaBlock DUT)
            {
                time = reference.time;

                if (!reference.valid || reference.fixType != FixType.Fix3D)
                {
                    valid = false;
                    device = Device.reference;
                    type = Type.invalid;
                    return;
                }

                if (!DUT.valid || DUT.fixType != FixType.Fix3D)
                {
                    valid = false;
                    device = Device.DUT_1;
                    type = Type.invalid;
                    return;
                }

                valid = true;

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

                maxSnr = relativeError(reference.satellites.Max((x) => x.SNR), DUT.satellites.Max((x) => x.SNR));
                maxSnrGPS = relativeError(reference.satellitesGPS.Max((x) => x.SNR), DUT.satellitesGPS.Max((x) => x.SNR));
                maxSnrGLONASS = relativeError(reference.satellitesGLONASS.Max((x) => x.SNR), DUT.satellitesGLONASS.Max((x) => x.SNR));
                avrSnr = relativeError(reference.satellites.Average((x) => x.SNR), DUT.satellites.Average((x) => x.SNR));
                avrSnrGPS = relativeError(reference.satellitesGPS.Average((x) => x.SNR), DUT.satellitesGPS.Average((x) => x.SNR));
                avrSnrGLONASS = relativeError(reference.satellitesGLONASS.Average((x) => x.SNR), DUT.satellitesGLONASS.Average((x) => x.SNR));

                IEnumerable<int> PRNs = reference.usedSatellites.Union(DUT.usedSatellites);
                SnrDevUsed = 0;
                foreach (int PRN in PRNs)
                {
                    double SNR_ref = reference.satellites.Where((x) => x.SNR != Double.NaN).Select((x) => x.PRN).Contains(PRN) ? reference.satellites.First(x => x.PRN == PRN).SNR : 0;
                    double SNR_DUT = DUT.satellites.Where((x) => x.SNR != Double.NaN).Select((x) => x.PRN).Contains(PRN) ? DUT.satellites.First(x => x.PRN == PRN).SNR : 0;
                    SnrDevUsed += relativeError(SNR_ref, SNR_DUT);
                }
                SnrDevUsed /= PRNs.Count();

                PRNs = reference.satellites.Where((x) => x.SNR != Double.NaN).Select((x) => x.PRN).Union(DUT.satellites.Where((x) => x.SNR != Double.NaN).Select((x) => x.PRN));
                SnrDevView = 0;
                foreach (int PRN in PRNs)
                {
                    double SNR_ref = reference.satellites.Where((x) => x.SNR != Double.NaN).Select((x) => x.PRN).Contains(PRN) ? reference.satellites.First(x => x.PRN == PRN).SNR : 0;
                    double SNR_DUT = DUT.satellites.Where((x) => x.SNR != Double.NaN).Select((x) => x.PRN).Contains(PRN) ? DUT.satellites.First(x => x.PRN == PRN).SNR : 0;
                    SnrDevView += relativeError(SNR_ref, SNR_DUT);
                }
                SnrDevView /= PRNs.Count();
            }
            private double relativeError(double x, double aprx)
            {
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

    }
}
