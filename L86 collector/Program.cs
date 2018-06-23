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
        private const string SoftwareVersion = "V1.1";

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

        class NmeaTestUnit
        {
            private bool init = false;
            private NmeaBlock nmeaBlock = null;
            private List<SatelliteData> satellitesGPS = null;
            private List<SatelliteData> satellitesGLONASS = null;
            private int GSAindex = 0;
            private readonly SerialPortDevice port;
            private readonly string rawFile;
            public readonly string designation;
            public ConcurrentQueue<NmeaBlock> queue;
            public readonly string portNum;
            public readonly string boardID;
            public readonly double boardHeight;
            public readonly double boardWidth;

            public NmeaTestUnit(string portNum, string rawFile, string designation, string boardID, double boardHeight, double boardWidth)
            {
                this.portNum = portNum;
                this.boardID = boardID;
                this.boardHeight = boardHeight;
                this.boardWidth = boardWidth;
                queue = new ConcurrentQueue<NmeaBlock>();
                port = new SerialPortDevice(new SerialPort("COM" + portNum, 9600, Parity.None, 8, StopBits.One));
                port.MessageReceived += NmeaMessageReceived;
                port.OpenAsync();
                this.rawFile = rawFile;
                this.designation = designation;
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
                writeToLogFile(message_.ToString() + "\r\n", rawFile, true);
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

        static NmeaTestUnit[] nmeaTestUnits;

        static void Main(string[] args)
        {
            #region start
            SetConsoleCtrlHandler(terminationEventHandler, true);


            Console.Write("Label of measurement: ");
            string lable = Console.ReadLine();
            Console.Write("Work directory: ");
#if DEBUG
            string folder = @"C:\Users\Adam\Desktop\GND Size Study\";
            Console.WriteLine(folder);
#else
            string folder = Console.ReadLine();
#endif
            string workFolder = Path.Combine(folder, lable) + @"\";
            if (Directory.Exists(workFolder))
            {
                Console.WriteLine("A measurement with this lable already exists in this directory!");
                Console.ReadLine();
                Environment.Exit(0);
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

            Console.Write("data acquisition device ID: ");
            string DAQ_ID = Console.ReadLine();

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
                nmeaTestUnitsList.Add(new NmeaTestUnit(ref_com, workFolder + lable + "_" + ref_des + "_REF.raw", ref_des, ref_boardID, ref_boardHeight, ref_boardWidth));
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
                    nmeaTestUnitsList.Add(new NmeaTestUnit(comNum, workFolder + lable + "_" + des + "_DUT.raw", des, boardID, boardHeight, boardWidth));
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
                                    writer.WriteString(nmeaTestUnits[0].portNum);
                                }
                                writer.WriteEndElement();

                                writer.WriteStartElement("board");
                                {
                                    writer.WriteStartElement("ID");
                                    {
                                        writer.WriteString(nmeaTestUnits[0].boardID);
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("heigh");
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
                                        writer.WriteString(nmeaTestUnits[i].portNum);
                                    }
                                    writer.WriteEndElement();

                                    writer.WriteStartElement("board");
                                    {
                                        writer.WriteStartElement("ID");
                                        {
                                            writer.WriteString(nmeaTestUnits[i].boardID);
                                        }
                                        writer.WriteEndElement();

                                        writer.WriteStartElement("heigh");
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

            try
            {
                using (StreamWriter write = new StreamWriter(workFolder + lable + ".dev", false))
                {
                    write.Write("time (UTC)");

                    for (int i = 1; i < nmeaTestUnits.Length; i++)
                    {
                        write.Write("\t{0}_offset\t{0}_PDOP\t{0}_HDOP\t{0}_VDOP\t{0}_satelliteInView\t{0}_satelliteInViewGPS\t{0}_satelliteInViewGLONASS\t{0}_satelliteUsed\t{0}_satelliteUsedGPS\t{0}_satelliteUsedGLONASS\t{0}_maxSnr\t{0}_maxSnrGPS\t{0}_maxSnrGLONASS\t{0}_avrSnr\t{0}_avrSnrGPS\t{0}_avrSnrGLONASS\t{0}_SnrDevView\t{0}_SnrDevUsed", nmeaTestUnits[i].designation);
                    }
                    write.WriteLine();
                }
            }
            catch (Exception)
            {
                Console.WriteLine(".dev file init error!");
                Console.ReadLine();
                Environment.Exit(0);
            }

            try
            {
                using (StreamWriter write = new StreamWriter(workFolder + lable + ".dat", false))
                {
                    write.Write("time (UTC)");

                    for (int i = 0; i < nmeaTestUnits.Length; i++)
                    {
                        write.Write("\t{0}_PDOP\t{0}_HDOP\t{0}_VDOP\t{0}_satelliteInView\t{0}_satelliteInViewGPS\t{0}_satelliteInViewGLONASS\t{0}_satelliteUsed\t{0}_satelliteUsedGPS\t{0}_satelliteUsedGLONASS\t{0}_maxSnr\t{0}_maxSnrGPS\t{0}_maxSnrGLONASS\t{0}_avrSnr\t{0}_avrSnrGPS\t{0}_avrSnrGLONASS", nmeaTestUnits[i].designation);
                    }
                    write.WriteLine();
                }
            }
            catch (Exception)
            {
                Console.WriteLine(".dat file init error!");
                Console.ReadLine();
                Environment.Exit(0);
            }

            try
            {
                using (StreamWriter write = new StreamWriter(workFolder + lable + ".err", false))
                    write.WriteLine("time (UTC)\tdevice\ttype\tcode");
            }
            catch (Exception)
            {
                Console.WriteLine(".err file init error!");
                Console.ReadLine();
                Environment.Exit(0);
            }

            try
            {
                using (StreamWriter write = new StreamWriter(workFolder + lable + ".loc", false))
                {
                    write.Write("time (UTC)");

                    for (int i = 0; i < nmeaTestUnits.Length; i++)
                    {
                        write.Write("\t{0}_latitude\t{0}_longitude\t{0}_altitude", nmeaTestUnits[i].designation);
                    }
                    write.WriteLine();
                }
            }
            catch (Exception)
            {
                Console.WriteLine(".loc file init error!");
                Console.ReadLine();
                Environment.Exit(0);
            }


            //empty queues
            foreach (NmeaTestUnit unit in nmeaTestUnits)
            {
                NmeaBlock trash;
                while (!unit.queue.IsEmpty)
                    while (!unit.queue.TryDequeue(out trash))
                        Task.Delay(1).Wait();
            }
            running = true;
            DateTime startTime = DateTime.UtcNow;
            #endregion

            #region new
            for (UInt64 i = 0; !terminationEventSignal.IsCancellationRequested; i++)
            {
                Console.Clear();
                Console.WriteLine("Number of datapoints: {0}\r\nElapsed time: {1:c}", i, DateTime.UtcNow.Subtract(startTime));

                NmeaBlock[] currentNmeaBlocks = new NmeaBlock[nmeaTestUnits.Length];
                DateTime currentTime = DateTime.UtcNow.AddYears(1);
                for (int j = 0; j < nmeaTestUnits.Length; j++)
                {
                    SpinWait.SpinUntil(() => nmeaTestUnits[j].queue.Count > 0);
                    while (!nmeaTestUnits[j].queue.TryPeek(out currentNmeaBlocks[j]))
                        Task.Delay(1).Wait();
                    currentTime = (currentTime > currentNmeaBlocks[j].time) ? currentNmeaBlocks[j].time : currentTime;
                }
                NmeaBlock refBlock = currentNmeaBlocks[0];

                string datF, devF, locF;
                datF = devF = locF = currentTime.ToString("yyyy/MM/dd HH:mm:ss");
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
                        }
                        else
                        {
                            if (!refBlock.valid) //invalid
                            {
                                errors += ", invalid";
                                code += 1 << 2;
                            }
                            else //no Fix3D
                            {
                                errors += ", " + (refBlock.fixType == FixType.Fix2D ? "Fix2D" : "noFix");
                                code += 1 << (refBlock.fixType == FixType.Fix2D ? 1 : 0);
                            }
                            while (!nmeaTestUnits[0].queue.TryDequeue(out currentNmeaBlocks[0]))
                                Task.Delay(1).Wait();
                        }
                        errF.Add(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[0].designation, errors, Convert.ToString(code, 2).PadLeft(5, '0')));
                        datF = stringMaker("\t", datF, (refBlock.fixType != FixType.Fix2D) ? datFileString(refBlock) : datFileString());
                        locF = stringMaker("\t", locF, (refBlock.fixType == FixType.Fix2D) ? locFileString(refBlock) : locFileString());
                        blockXmler(refBlock, logFXmlWriter, nmeaTestUnits[0].designation, "reference", 0, nmeaTestUnits[0].boardID, nmeaTestUnits[0].boardHeight, nmeaTestUnits[0].boardWidth);
                    }
                    else
                    {
                        while (!nmeaTestUnits[0].queue.TryDequeue(out currentNmeaBlocks[0]))
                            Task.Delay(1).Wait();
                        datF = stringMaker("\t", datF, datFileString(refBlock));
                        locF = stringMaker("\t", locF, locFileString(refBlock));
                        blockXmler(refBlock, logFXmlWriter, nmeaTestUnits[0].designation, "reference", 0, nmeaTestUnits[0].boardID, nmeaTestUnits[0].boardHeight, nmeaTestUnits[0].boardWidth);
                    }
                    for (int j = 1; j < nmeaTestUnits.Length; j++)
                    {
                        NmeaBlock block = currentNmeaBlocks[j];
                        if (!block.valid || (block.time != currentTime) || (block.fixType != FixType.Fix3D))
                        {
                            string errorStr;
                            int code;
                            if (block.time != currentTime) //missing
                            {
                                errorStr = "missing";
                                code = 1 << 3;
                            }
                            else
                            {
                                if (!block.valid) //invalid
                                {
                                    errorStr = "invalid";
                                    code = 1 << 2;
                                }
                                else //no Fix3D
                                {
                                    errorStr = (block.fixType == FixType.Fix2D ? "Fix2D" : "noFix");
                                    code = 1 << (block.fixType == FixType.Fix2D ? 1 : 0);
                                }
                                while (!nmeaTestUnits[j].queue.TryDequeue(out currentNmeaBlocks[j]))
                                    Task.Delay(1).Wait();
                            }
                            errF.Add(stringMaker("\t", currentTime.ToString("yyyy/MM/dd HH:mm:ss"), nmeaTestUnits[j].designation, errorStr, Convert.ToString(code, 2).PadLeft(5, '0')));
                            datF = stringMaker("\t", datF, (block.fixType != FixType.Fix2D) ? datFileString(block) : datFileString());
                            locF = stringMaker("\t", locF, (block.fixType == FixType.Fix2D) ? locFileString(block) : locFileString());
                            devF = stringMaker("\t", devF, devFileString());
                            blockXmler(block, logFXmlWriter, nmeaTestUnits[j].designation, "DUT", j, nmeaTestUnits[j].boardID, nmeaTestUnits[j].boardHeight, nmeaTestUnits[j].boardWidth);
                        }
                        else
                        {
                            while (!nmeaTestUnits[j].queue.TryDequeue(out currentNmeaBlocks[j]))
                                Task.Delay(1).Wait();
                            datF = stringMaker("\t", datF, datFileString(block));
                            locF = stringMaker("\t", locF, locFileString(block));
                            devF = stringMaker("\t", devF, refValid ? devFileString(refBlock, currentNmeaBlocks[j]) : devFileString());
                            blockXmler(block, logFXmlWriter, nmeaTestUnits[j].designation, "DUT", j, nmeaTestUnits[j].boardID, nmeaTestUnits[j].boardHeight, nmeaTestUnits[j].boardWidth);
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
                        writeToLogFile(devF + "\r\n", workFolder + lable + ".dev", true);
                    writeToLogFile(datF + "\r\n", workFolder + lable + ".dat", true);
                    writeToLogFile(locF + "\r\n", workFolder + lable + ".loc", true);
                    writeToLogFile(errF, workFolder + lable + ".err", true);
                    writeToLogFile(logF.ToString(), workFolder + lable + ".log", true);
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
            return "-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-";
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
            return "-\t-\t-";
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
            return "-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-";
        }

        static string stringMaker(string separator, params string[] strings)
        {
            return String.Join(separator, strings);
        }

        static void writeToLogFile(string text, string fileName, bool append)
        {
            for (int i = 0; i < 6000; i++)
            {
                try
                {
                    using (StreamWriter write = new StreamWriter(fileName, append))
                    {
                        write.Write(text);
                    }
                    return;
                }
                catch (Exception)
                {
                    Task.Delay(10).Wait();
                }
            }
            throw new Exception("writeToLogFile failed");
        }
        static void writeToLogFile(string[] texts, string fileName, bool append)
        {
            for (int i = 0; i < 6000; i++)
            {
                try
                {
                    using (StreamWriter write = new StreamWriter(fileName, append))
                    {
                        write.WriteLine(String.Join("\r\n", texts));
                    }
                    return;
                }
                catch (Exception)
                {
                    Task.Delay(10).Wait();
                }
            }
            throw new Exception("writeToLogFile failed");
        }

        static void writeToLogFile(List<string> texts, string fileName, bool append)
        {
            for (int i = 0; i < 6000; i++)
            {
                try
                {
                    using (StreamWriter write = new StreamWriter(fileName, append))
                    {
                        write.WriteLine(String.Join("\r\n", texts));
                    }
                    return;
                }
                catch (Exception)
                {
                    Task.Delay(10).Wait();
                }
            }
            throw new Exception("writeToLogFile failed");
        }

        static void blockXmler(NmeaBlock block, XmlTextWriter writer, string deviceDesignation, string deviceRole, int deviceID, string boardID, double boardHeigth, double boardWidth)
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

                    writer.WriteStartElement("heigh");
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
