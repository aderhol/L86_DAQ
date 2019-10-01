//#define NMEADevice_UnitTest

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.IO.Ports;
using System.IO;
using System.Collections.Concurrent;
using System.Diagnostics;
using CustumLoggers;

namespace NMEA_Parser
{
    #region Nmea
    public class NmeaMessage
    {
        public string MessageType { get; }
        private string RawMessage;
        public override string ToString()
        {
            return RawMessage;
        }
        protected NmeaMessage(string rawMessage, string messageType)
        {
            RawMessage = rawMessage;
            MessageType = messageType;
        }
        protected NmeaMessage(NmeaMessage x)
        {
            RawMessage = x.RawMessage;
            MessageType = x.MessageType;
        }
    }

    public class Gptxt : NmeaMessage
    {
        public Gptxt(string rawMessage) : base(rawMessage, "GPTXT")
        {

        }
    }
    public class Gntxt : NmeaMessage
    {
        public Gntxt(string rawMessage) : base(rawMessage, "GNTXT")
        {

        }
    }
    public class Gnzda : NmeaMessage
    {
        public Gnzda(string rawMessage) : base(rawMessage, "GNZDA")
        {

        }
    }

    public class Unknown : NmeaMessage
    {
        public Unknown(string rawMessage) : base(rawMessage, "Unknown")
        {

        }
    }

    public class Rmc : NmeaMessage
    {
        public DateTime FixTime { get; }
        public bool Active { get; }
        public double Latitude { get; }
        public double Longitude { get; }
        public double Speed { get; }
        public double Course { get; }

        protected Rmc(string rawMessage, string messageType, DateTime fixTime, bool active, double latitude, double longitude, double speed, double course) : base(rawMessage, messageType)
        {
            FixTime = fixTime;
            Active = active;
            Latitude = latitude;
            Longitude = longitude;
            Speed = speed;
            Course = course;
        }

        protected Rmc(Rmc x) : base(x)
        {
            FixTime = x.FixTime;
            Active = x.Active;
            Latitude = x.Latitude;
            Longitude = x.Longitude;
            Speed = x.Speed;
            Course = x.Course;
        }
    }
    public class Gprmc : Rmc
    {
        public Gprmc(string rawMessage, DateTime fixTime, bool active, double latitude, double longitude, double speed, double course) : base(rawMessage, "GPRMC", fixTime, active, latitude, longitude, speed, course)
        {
        }
        public Gprmc(Gprmc x) : base(x)
        { }
    }
    public class Gnrmc : Rmc
    {
        public Gnrmc(string rawMessage, DateTime fixTime, bool active, double latitude, double longitude, double speed, double course) : base(rawMessage, "GNRMC", fixTime, active, latitude, longitude, speed, course)
        {
        }
        public Gnrmc(Gnrmc x) : base(x)
        { }
    }

    public enum FixQualityEnum
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
    public class Gga : NmeaMessage
    {
        public FixQualityEnum Quality { get; }
        public int NumberOfSatellites { get; }
        public double Altitude { get; }
        public TimeSpan TimeSinceLastDgpsUpdate { get; }
        public int DgpsStationId { get; }

        protected Gga(string rawMessage, string messageType, FixQualityEnum quality, int numberOfSatellites, double altitude, TimeSpan timeSinceLastDgpsUpdate, int dgpsStationId) : base(rawMessage, messageType)
        {
            Quality = quality;
            NumberOfSatellites = numberOfSatellites;
            Altitude = altitude;
            TimeSinceLastDgpsUpdate = timeSinceLastDgpsUpdate;
            DgpsStationId = dgpsStationId;
        }
        protected Gga(Gga x) : base(x)
        {
            Quality = x.Quality;
            NumberOfSatellites = x.NumberOfSatellites;
            Altitude = x.Altitude;
            TimeSinceLastDgpsUpdate = x.TimeSinceLastDgpsUpdate;
            DgpsStationId = x.DgpsStationId;
        }
    }
    public class Gpgga : Gga
    {
        public Gpgga(string rawMessage, FixQualityEnum quality, int numberOfSatellites, double altitude, TimeSpan timeSinceLastDgpsUpdate, int dgpsStationId) : base(rawMessage, "GPGGA", quality, numberOfSatellites, altitude, timeSinceLastDgpsUpdate, dgpsStationId)
        { }
        public Gpgga(Gpgga x) : base(x)
        { }
    }
    public class Gngga : Gga
    {
        public Gngga(string rawMessage, FixQualityEnum quality, int numberOfSatellites, double altitude, TimeSpan timeSinceLastDgpsUpdate, int dgpsStationId) : base(rawMessage, "GNGGA", quality, numberOfSatellites, altitude, timeSinceLastDgpsUpdate, dgpsStationId)
        {
        }
        public Gngga(Gngga x) : base(x)
        { }
    }

    public enum Mode
    {
        noFix = 1,
        Fix2D,
        Fix3D
    }
    public class Gsa : NmeaMessage
    {
        public Mode FixMode { get; }
        public double Hdop { get; }
        public double Vdop { get; }
        public double Pdop { get; }
        private List<int> _SVs;
        public IReadOnlyList<int> SVs { get { return _SVs; } }

        protected Gsa(string rawMessage, string messageType, Mode fixMode, double hdop, double vdop, double pdop, List<int> sVs) : base(rawMessage, messageType)
        {
            FixMode = fixMode;
            Hdop = hdop;
            Vdop = vdop;
            Pdop = pdop;
            _SVs = new List<int>(sVs);
        }
        protected Gsa(Gsa x) : base(x)
        {
            FixMode = x.FixMode;
            Hdop = x.Hdop;
            Vdop = x.Vdop;
            Pdop = x.Pdop;
            _SVs = new List<int>(x._SVs);
        }
    }
    public class Gpgsa : Gsa
    {
        public Gpgsa(string rawMessage, Mode fixMode, double hdop, double vdop, double pdop, List<int> sVs) : base(rawMessage, "GPGSA", fixMode, hdop, vdop, pdop, sVs)
        {
        }
        public Gpgsa(Gpgsa x) : base(x)
        {

        }
    }
    public class Gngsa : Gsa
    {
        public Gngsa(string rawMessage, Mode fixMode, double hdop, double vdop, double pdop, List<int> sVs) : base(rawMessage, "GNGSA", fixMode, hdop, vdop, pdop, sVs)
        {
        }
        public Gngsa(Gngsa x) : base(x)
        { }
    }

    public class SatelliteVehicle
    {
        public int PrnNumber { get; }
        public double Elevation { get; }
        public double Azimuth { get; }
        public int SignalToNoiseRatio { get; }

        public SatelliteVehicle(int prnNumber, double elevation, double azimuth, int signalToNoiseRatio)
        {
            PrnNumber = prnNumber;
            Elevation = elevation;
            Azimuth = azimuth;
            SignalToNoiseRatio = signalToNoiseRatio;
        }
        public SatelliteVehicle(SatelliteVehicle x)
        {
            PrnNumber = x.PrnNumber;
            Elevation = x.Elevation;
            Azimuth = x.Azimuth;
            SignalToNoiseRatio = x.SignalToNoiseRatio;
        }
    }
    public class Gsv : NmeaMessage
    {
        public int SVsInView { get; }
        private List<SatelliteVehicle> _SVs;
        public IReadOnlyList<SatelliteVehicle> SVs { get { return _SVs; } }

        protected Gsv(string rawMessage, string messageType, int sVsInView, List<SatelliteVehicle> sVs) : base(rawMessage, messageType)
        {
            SVsInView = sVsInView;
            _SVs = new List<SatelliteVehicle>();
            foreach (var satelliteVehicle in sVs)
            {
                _SVs.Add(new SatelliteVehicle(satelliteVehicle));
            }
        }
        protected Gsv(Gsv x) : base(x)
        {
            SVsInView = x.SVsInView;
            _SVs = new List<SatelliteVehicle>();
            foreach (var satelliteVehicle in x.SVs)
            {
                _SVs.Add(new SatelliteVehicle(satelliteVehicle));
            }
        }
    }
    public class Gpgsv : Gsv
    {
        public Gpgsv(string rawMessage, int sVsInView, List<SatelliteVehicle> sVs) : base(rawMessage, "GPGSV", sVsInView, sVs)
        {
        }
        public Gpgsv(Gpgsv x) : base(x)
        {
        }
    }
    public class Glgsv : Gsv
    {
        public Glgsv(string rawMessage, int sVsInView, List<SatelliteVehicle> sVs) : base(rawMessage, "GLGSV", sVsInView, sVs)
        {
        }
        public Glgsv(Glgsv x) : base(x)
        {
        }
    }
    public class Gagsv : Gsv
    {
        public Gagsv(string rawMessage, int sVsInView, List<SatelliteVehicle> sVs) : base(rawMessage, "GAGSV", sVsInView, sVs)
        {
        }
        public Gagsv(Gagsv x) : base(x)
        {
        }
    }

    public enum PpsStatus
    {
        OK,
        notPossible,
        glitch,
        old,
        ppsMissing,
        refMissing,
        unexpected
    }
    public class Refcheck : NmeaMessage
    {
        public double softwareDelay { get; }
        public double skew_ns { get; }
        public Int64 absTime_ref_ticks { get; }
        public Int64 absTime_check_ticks { get; }
        public PpsStatus status { get; }

        public Refcheck(string rawMessage, double softwareDelay, double skew_ns, Int64 absTime_ref_ticks, Int64 absTime_check_ticks, PpsStatus status) : base(rawMessage, "REFCHECK")
        {
            this.softwareDelay = softwareDelay;
            this.skew_ns = skew_ns;
            this.absTime_ref_ticks = absTime_ref_ticks;
            this.absTime_check_ticks = absTime_check_ticks;
            this.status = status;
        }
    }
    public class Reftime : NmeaMessage
    {
        public Int64 absTime_ticks { get; }

        public Reftime(string rawMessage, Int64 absTime_ticks) : base(rawMessage, "REFTIME")
        {
            this.absTime_ticks = absTime_ticks;
        }
    }

    public class Extraref : NmeaMessage
    {
        public string channelName { get; }
        public Extraref(string rawMessage, string channelName) : base(rawMessage, "EXTRAREF")
        {
            this.channelName = channelName;
        }
    }

    public class Skew : NmeaMessage
    {
        public double packetDelay { get; }
        public double skew_ns { get; }
        public Int64 absTime_ticks { get; }
        public PpsStatus status { get; }

        public Skew(string rawMessage, double packetDelay, double skew_ns, Int64 absTime_ticks, PpsStatus status) : base(rawMessage, "SKEW")
        {
            this.packetDelay = packetDelay;
            this.skew_ns = skew_ns;
            this.absTime_ticks = absTime_ticks;
            this.status = status;
        }
    }

    public enum SenDataStatus
    {
        OK,
        old,
        invalid
    }
    public class SenData : NmeaMessage
    {
        public double temperature_bmp180_C { get; }
        public double pressure_Pa { get; }

        public double visibleLightIntensity_lx { get; }
        public double infraredLighIntensity { get; }

        public double temperature_sht21_C { get; }
        public double relativeHumidity_percentage { get; }

        public double sampleAge_ms { get; }

        public SenDataStatus status { get; }

        public SenData(string rawMessage, double temperature_bmp180_C, double pressure_Pa, double visibleLightIntensity_lx, double infraredLighIntensity, double temperature_C_sht21, double relativeHumidity_percentage, double samlpleAge_ms, SenDataStatus status) : base(rawMessage, "SENDAT")
        {
            this.temperature_bmp180_C = temperature_bmp180_C;
            this.pressure_Pa = pressure_Pa;
            this.visibleLightIntensity_lx = visibleLightIntensity_lx;
            this.infraredLighIntensity = infraredLighIntensity;
            this.temperature_sht21_C = temperature_C_sht21;
            this.relativeHumidity_percentage = relativeHumidity_percentage;
            this.sampleAge_ms = samlpleAge_ms;
            this.status = status;
        }
    }

    public class PpsInfo : NmeaMessage
    {
        public double delay { get; }
        public double average_90 { get; }
        public double average_500 { get; }
        public double average_1000 { get; }
        public double temperature { get; }

        public PpsInfo(string rawMessage, double delay, double delay_A90, double delay_A500, double delay_A1000, double temperature) : base(rawMessage, "GPINF")
        {
            this.delay = delay / 2;

            this.average_90 = delay_A90 / 2;
            this.average_500 = delay_A500 / 2;
            this.average_1000 = delay_A1000 / 2;

            this.temperature = temperature;
        }
    }


    public class NmeaMessageReceivedEventArgs : EventArgs
    {
        public NmeaMessage Message { get; }
        public NmeaMessageReceivedEventArgs(NmeaMessage message)
        {
            Message = message;
        }
    }

    #endregion //NMEA

    public class NmeaDevice
    {
        private volatile SerialPort Port;
        public event EventHandler MessageReceived;
        private volatile string Incoming;
        private volatile ConcurrentQueue<string> IncomingQueue;
        private Thread Collector;
        private Thread SerialProcessor;

        private ThreadedLogger logger;

        private StreamReader inputFile;
        private AutoResetEvent ResetRequested = new AutoResetEvent(false);
        public event EventHandler FileProcessed;



        public NmeaDevice(SerialPort serialPort, string rawFilePath, string name)
        {
            if (File.Exists(rawFilePath))
                throw new Exception("NmeaDevice: rawFile exists");
            if (!Directory.Exists(new FileInfo(rawFilePath).Directory.FullName))
                throw new Exception("NmeaDevice: rawFile directory does not exists");


            logger = new ThreadedLogger(rawFilePath, "Logger: " + name);

            Port = serialPort;
            Incoming = "";
            IncomingQueue = new ConcurrentQueue<string>();

            Collector = new Thread(new ThreadStart(SerialCollectorWorker));
            Collector.Name = "SerialCollector: " + name;
            Collector.IsBackground = true;

            SerialProcessor = new Thread(new ThreadStart(Process));
            SerialProcessor.Name = "SerialProcessor: " + name;
            SerialProcessor.IsBackground = true;
        }

        public NmeaDevice(string inputFilePath, string rawFilePath, string name)
        {
            if (File.Exists(rawFilePath))
                throw new Exception("NmeaDevice: rawFile exists");
            if (!Directory.Exists(new FileInfo(rawFilePath).Directory.FullName))
                throw new Exception("NmeaDevice: rawFile directory does not exists");
            if (!File.Exists(inputFilePath))
                throw new Exception("NmeaDevice: inputFile does not exists");


            logger = new ThreadedLogger(rawFilePath, "Logger: " + name);

            inputFile = new StreamReader(inputFilePath);
            IncomingQueue = new ConcurrentQueue<string>();

            Collector = new Thread(new ThreadStart(FileCollectorWorker));
            Collector.Name = "FileCollector: " + name;
            Collector.IsBackground = true;

            SerialProcessor = new Thread(new ThreadStart(Process));
            SerialProcessor.Name = "FileProcessor: " + name;
            SerialProcessor.IsBackground = true;

            Port = null;
        }

#if NMEADevice_UnitTest
        public NmeaDevice(SerialPort serialPort, string name)
        {
            Port = serialPort;
            Incoming = "";
            IncomingQueue = new ConcurrentQueue<string>();

            SerialCollector = new Thread(new ThreadStart(SerialCollectorWorker));
            SerialCollector.Name = "SerialCollector: " + name;
            SerialCollector.IsBackground = true;

            SerialProcessor = new Thread(new ThreadStart(Process));
            SerialProcessor.Name = "SeralProcessor: " + name;
            SerialProcessor.IsBackground = true;
        }
#endif

        public void OpenPort()
        {
            if (Port != null && Port.IsOpen)
                throw new Exception("NMEA_Parser: double port open");

            Collector.Start();
            SerialProcessor.Start();
        }

        ManualResetEvent collectorStopped = new ManualResetEvent(false);
        private void SerialCollectorWorker()
        {
            Port.ReadBufferSize = 51200;
            Port.Open();

            int contCount = 0;
            while (!isCloseRequested.IsCancellationRequested)
            {
                Thread.Sleep(100);
                if (Port.BytesToRead < 30 && (contCount < 5 || Port.BytesToRead == 0))
                {
                    contCount++;
                    continue;
                }

                contCount = 0;
                string snippet = Port.ReadExisting();
#if !NMEADevice_UnitTest
                logger.Log(snippet);
#endif
                IncomingQueue.Enqueue(snippet);
            }
            collectorStopped.Set();
        }

        private void FileCollectorWorker()
        {
            ResetRequested.WaitOne();

            bool endOfFile = false;

            int delayCnt = 0;

            while (!isCloseRequested.IsCancellationRequested)
            {
                if (IncomingQueue.Count > 10)
                {
                    Thread.Sleep(delayCnt);

                    if (delayCnt < 100)
                        delayCnt++;

                    continue;
                }
                else
                {
                    delayCnt = 0;
                }

                char[] snippetBuff = new char[1000];
                int cnt = inputFile.ReadBlock(snippetBuff, 0, snippetBuff.Length);
                string snippet = new string(snippetBuff, 0, cnt);

#if !NMEADevice_UnitTest
                logger.Log(snippet);
#endif
                IncomingQueue.Enqueue(snippet);

                if (cnt < snippetBuff.Length || inputFile.EndOfStream)
                {
                    endOfFile = true;
                    break;
                }
            }


            inputFile.Close();

            collectorStopped.Set();

            if (endOfFile)
                FileProcessed?.Invoke(this, new EventArgs());
        }

        private int Gpgsv_count = 0;
        private int Gpgsv_errCount = 0;
        private List<SatelliteVehicle> Gpgsv_sat = new List<SatelliteVehicle>();
        private string GpgsvRaw = "";
        private int Glgsv_count = 0;
        private int Glgsv_errCount = 0;
        private List<SatelliteVehicle> Glgsv_sat = new List<SatelliteVehicle>();
        private string GlgsvRaw = "";
        private int Gagsv_count = 0;
        private int Gagsv_errCount = 0;
        private List<SatelliteVehicle> Gagsv_sat = new List<SatelliteVehicle>();
        private string GagsvRaw = "";
        private void Process()
        {
            while (true)
            {
                while (IncomingQueue.IsEmpty)
                    Thread.Sleep(100);

                for (int i = IncomingQueue.Count; i > 0; i--) //doesn't allow starving
                {
                    string rawSerialData;
                    while (!IncomingQueue.TryDequeue(out rawSerialData))
                        Thread.Yield();

                    if (Incoming == "")
                    {
                        Incoming = new string(rawSerialData.SkipWhile((x) => x != '$').ToArray());
                    }
                    else
                    {
                        Incoming += rawSerialData;
                    }
                }

                string raw, message;

                while (Incoming.Contains("\n"))
                {
                    raw = new string(Incoming.TakeWhile((x) => x != '\n').ToArray()) + "\n"; //fetch message
                    Incoming = Incoming.Remove(0, raw.Length); //remove fetched message
                    raw = raw.TrimEnd("\r\n".ToCharArray());

                    if (raw.Length == 0 || raw[0] != '$') //if the message is empty or is not an NMEA message
                    {
                        continue;
                    }

                    int starIndex = raw.IndexOf('*');
                    if (starIndex == -1)
                        continue;   //the char * was not found
                    byte checksum_ref;
                    if (!byte.TryParse(raw.Substring(starIndex + 1, 2), System.Globalization.NumberStyles.HexNumber, null, out checksum_ref))
                        continue;

                    message = raw.Substring(1, starIndex - 1);
                    byte checksum = 0;
                    for (int i = 0; i < message.Length; i++)
                    {
                        checksum ^= (byte)message[i];
                    }
                    if (checksum != checksum_ref)
                        continue;

                    string[] tokens = message.Split(',');

                    if (tokens.Length == 0)
                        continue;

                    if (tokens[0] == "GPGSV")
                    {
                        GpgsvRaw += ((GpgsvRaw == "") ? ("") : ("\r\n")) + raw;
                        Gpgsv_count++;
                    }
                    else if (tokens[0][0] == 'G' && (tokens[0][1] == 'P' || tokens[0][1] == 'L' || tokens[0][1] == 'N' || tokens[0][1] == 'A')) //if real NMEA message
                    {
                        Gpgsv_count = 0;
                        Gpgsv_errCount = 0;
                        Gpgsv_sat = new List<SatelliteVehicle>();
                        GpgsvRaw = "";
                    }

                    if (tokens[0] == "GLGSV")
                    {
                        GlgsvRaw += ((GlgsvRaw == "") ? ("") : ("\r\n")) + raw;
                        Glgsv_count++;
                    }
                    else if (tokens[0][0] == 'G' && (tokens[0][1] == 'P' || tokens[0][1] == 'L' || tokens[0][1] == 'N' || tokens[0][1] == 'A')) //if real NMEA message
                    {
                        Glgsv_count = 0;
                        Glgsv_errCount = 0;
                        Glgsv_sat = new List<SatelliteVehicle>();
                        GlgsvRaw = "";
                    }

                    if (tokens[0] == "GAGSV")
                    {
                        GagsvRaw += ((GagsvRaw == "") ? ("") : ("\r\n")) + raw;
                        Gagsv_count++;
                    }
                    else if (tokens[0][0] == 'G' && (tokens[0][1] == 'P' || tokens[0][1] == 'L' || tokens[0][1] == 'N' || tokens[0][1] == 'A')) //if real NMEA message
                    {
                        Gagsv_count = 0;
                        Gagsv_errCount = 0;
                        Gagsv_sat = new List<SatelliteVehicle>();
                        GagsvRaw = "";
                    }

                    switch (tokens[0])
                    {
                        case "GPTXT":
                            MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gptxt(raw)));
                            break;
                        case "GNTXT":
                            MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gntxt(raw)));
                            break;
                        case "GNZDA":
                            MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gnzda(raw)));
                            break;

                        case "GPRMC":
                            {
                                if (tokens.Length != 13 && tokens.Length != 14) //14 if NMEA 4.1 and above
                                    break;

                                bool v4 = tokens.Length == 14; //indicates a version 4 NMEA message


                                DateTime dateTime;
                                try
                                {
                                    if (tokens[1] != "" && tokens[9] != "")
                                    {
                                        if (v4)
                                        {
                                            dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.ff ddMMyy", null);
                                        }
                                        else
                                        {
                                            if (tokens[1].Length == 10)
                                                dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.fff ddMMyy", null);
                                            else
                                                dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.ff ddMMyy", null);
                                        }
                                    }
                                    else
                                    {
                                        dateTime = DateTime.MinValue;
                                    }
                                }
                                catch (Exception el)
                                {
                                    throw el;
                                }

                                bool active;
                                if (tokens[2] == "A")
                                    active = true;
                                else if (tokens[2] == "V")
                                    active = false;
                                else
                                    throw new Exception("GPRMC: active");

                                double latitude, lat_min, lat_sec;
                                if (tokens[3].Length >= 3 && double.TryParse(tokens[3].Substring(0, 2), out lat_min) && double.TryParse(tokens[3].Substring(2), out lat_sec))
                                    latitude = (lat_min + (lat_sec / 60)) * (tokens[4] == "N" ? 1 : -1);
                                else
                                    latitude = double.NaN;

                                double longitude, lon_min, lon_sec;
                                if (tokens[5].Length >= 4 && double.TryParse(tokens[5].Substring(0, 3), out lon_min) && double.TryParse(tokens[5].Substring(3), out lon_sec))
                                    longitude = (lon_min + (lon_sec / 60)) * (tokens[6] == "E" ? 1 : -1);
                                else
                                    longitude = double.NaN;

                                double speed;
                                if (!double.TryParse(tokens[7], out speed))
                                    speed = double.NaN;

                                double course;
                                if (!double.TryParse(tokens[8], out course))
                                    course = double.NaN;

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gprmc(raw, dateTime, active, latitude, longitude, speed, course)));
                            }
                            break;

                        case "GNRMC":
                            {
                                if (tokens.Length != 13 && tokens.Length != 14) //14 if NMEA 4.1 and above
                                    break;

                                bool v4 = tokens.Length == 14; //indicates a version 4 NMEA message

                                DateTime dateTime;
                                try
                                {
                                    if (tokens[1] != "" && tokens[9] != "")
                                    {
                                        if (v4)
                                        {
                                            dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.ff ddMMyy", null);
                                        }
                                        else
                                        {
                                            if (tokens[1].Length == 10)
                                                dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.fff ddMMyy", null);
                                            else
                                                dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.ff ddMMyy", null);
                                        }
                                    }
                                    else
                                    {
                                        dateTime = DateTime.MinValue;
                                    }
                                }
                                catch (Exception el)
                                {
                                    throw el;
                                }

                                bool active;
                                if (tokens[2] == "A")
                                    active = true;
                                else if (tokens[2] == "V")
                                    active = false;
                                else
                                    throw new Exception("GNRMC: active");

                                double latitude, lat_min, lat_sec;
                                if (tokens[3].Length >= 3 && double.TryParse(tokens[3].Substring(0, 2), out lat_min) && double.TryParse(tokens[3].Substring(2), out lat_sec))
                                    latitude = (lat_min + (lat_sec / 60)) * (tokens[4] == "N" ? 1 : -1);
                                else
                                    latitude = double.NaN;

                                double longitude, lon_min, lon_sec;
                                if (tokens[5].Length >= 4 && double.TryParse(tokens[5].Substring(0, 3), out lon_min) && double.TryParse(tokens[5].Substring(3), out lon_sec))
                                    longitude = (lon_min + (lon_sec / 60)) * (tokens[6] == "E" ? 1 : -1);
                                else
                                    longitude = double.NaN;

                                double speed;
                                if (!double.TryParse(tokens[7], out speed))
                                    speed = double.NaN;

                                double course;
                                if (!double.TryParse(tokens[8], out course))
                                    course = double.NaN;
                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gnrmc(raw, dateTime, active, latitude, longitude, speed, course)));
                            }
                            break;

                        case "GPGGA":
                            {
                                if (tokens.Length != 15)
                                    break;
                                FixQualityEnum quality;
                                int q_vessel;
                                try
                                {
                                    q_vessel = Convert.ToInt32(tokens[6]);
                                    if (q_vessel > 8 || q_vessel < 0)
                                        throw new Exception("GPGGA:quality");
                                    quality = (FixQualityEnum)q_vessel;
                                }
                                catch
                                {
                                    throw;
                                }

                                int numberOfSatellites;
                                try
                                {
                                    numberOfSatellites = Convert.ToInt32(tokens[7]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double altitude;
                                if (!double.TryParse(tokens[9], out altitude))
                                    altitude = double.NaN;

                                TimeSpan timeSinceLastDgpsUpdate;
                                double time;
                                if (double.TryParse(tokens[13], out time))
                                    timeSinceLastDgpsUpdate = new TimeSpan(0, 0, 0, (int)time, (int)(time - Math.Floor(time)));
                                else
                                    timeSinceLastDgpsUpdate = TimeSpan.MaxValue;


                                int dgpsStationId;
                                if (!int.TryParse(tokens[14], out dgpsStationId))
                                    dgpsStationId = -1;

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gpgga(raw, quality, numberOfSatellites, altitude, timeSinceLastDgpsUpdate, dgpsStationId)));
                            }
                            break;

                        case "GNGGA":
                            {
                                if (tokens.Length != 15)
                                    break;
                                FixQualityEnum quality;
                                int q_vessel;
                                try
                                {
                                    q_vessel = Convert.ToInt32(tokens[6]);
                                    if (q_vessel > 8 || q_vessel < 0)
                                        throw new Exception("GNGGA:quality");
                                    quality = (FixQualityEnum)q_vessel;
                                }
                                catch
                                {
                                    throw;
                                }

                                int numberOfSatellites;
                                try
                                {
                                    numberOfSatellites = Convert.ToInt32(tokens[7]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double altitude;
                                if (!double.TryParse(tokens[9], out altitude))
                                    altitude = double.NaN;

                                TimeSpan timeSinceLastDgpsUpdate;
                                double time;
                                if (double.TryParse(tokens[13], out time))
                                    timeSinceLastDgpsUpdate = new TimeSpan(0, 0, 0, (int)time, (int)(time - Math.Floor(time)));
                                else
                                    timeSinceLastDgpsUpdate = TimeSpan.MaxValue;


                                int dgpsStationId;
                                if (!int.TryParse(tokens[14], out dgpsStationId))
                                    dgpsStationId = -1;

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gngga(raw, quality, numberOfSatellites, altitude, timeSinceLastDgpsUpdate, dgpsStationId)));
                            }
                            break;

                        case "GNGSA":
                            {
                                if (tokens.Length != 18 && tokens.Length != 19) //19 if NMEA 4.1 or above
                                    break;

                                Mode fixMode;
                                int f_vessel;
                                try
                                {
                                    f_vessel = Convert.ToInt32(tokens[2]);
                                    if (f_vessel > 3 || f_vessel < 1)
                                        throw new Exception("GNGSA:FixMode");
                                    fixMode = (Mode)f_vessel;
                                }
                                catch
                                {
                                    throw;
                                }

                                double hdop;
                                if (!double.TryParse(tokens[16], out hdop))
                                    hdop = double.NaN;

                                double vdop;
                                if (!double.TryParse(tokens[17], out vdop))
                                    vdop = double.NaN;

                                double pdop;
                                if (!double.TryParse(tokens[15], out pdop))
                                    pdop = double.NaN;

                                List<int> sVs = new List<int>();
                                for (int i = 3; (i <= 14) && (tokens[i] != ""); i++)
                                {
                                    try
                                    {
                                        sVs.Add(Convert.ToInt32(tokens[i]));
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                }

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gngsa(raw, fixMode, hdop, vdop, pdop, sVs)));
                            }
                            break;

                        case "GPGSA":
                            {
                                if (tokens.Length != 18 && tokens.Length != 19) //19 if NMEA 4.1 or above
                                    break;

                                Mode fixMode;
                                int f_vessel;
                                try
                                {
                                    f_vessel = Convert.ToInt32(tokens[2]);
                                    if (f_vessel > 3 || f_vessel < 1)
                                        throw new Exception("GPGSA:FixMode");
                                    fixMode = (Mode)f_vessel;
                                }
                                catch
                                {
                                    throw;
                                }

                                double hdop;
                                if (!double.TryParse(tokens[16], out hdop))
                                    hdop = double.NaN;

                                double vdop;
                                if (!double.TryParse(tokens[17], out vdop))
                                    vdop = double.NaN;

                                double pdop;
                                if (!double.TryParse(tokens[15], out pdop))
                                    pdop = double.NaN;

                                List<int> sVs = new List<int>();
                                for (int i = 3; (i <= 14) && (tokens[i] != ""); i++)
                                {
                                    try
                                    {
                                        sVs.Add(Convert.ToInt32(tokens[i]));
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                }

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gpgsa(raw, fixMode, hdop, vdop, pdop, sVs)));
                            }
                            break;

                        case "GPGSV":
                            {
                                if (tokens.Length < 4)
                                    break;

                                if (tokens.Length % 4 == 1)
                                    tokens = tokens.Take(tokens.Length - 1).ToArray();

                                int seqNum;
                                try
                                {
                                    seqNum = Convert.ToInt32(tokens[2]);
                                }
                                catch
                                {
                                    throw;
                                }
                                if (seqNum != Gpgsv_count)
                                {
                                    break;
                                }

                                int totalNum;
                                try
                                {
                                    totalNum = Convert.ToInt32(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                for (int i = 4; i < tokens.Length; i += 4)
                                {
                                    try
                                    {
                                        if (tokens[i] == "")
                                        {
                                            if (tokens[i + 3] != "")
                                            {
                                                Gpgsv_errCount++;
                                            }
                                            continue;
                                        }

                                        Gpgsv_sat.Add(new SatelliteVehicle(Convert.ToInt32(tokens[i]), (tokens[i + 1] == "") ? double.NaN : Convert.ToDouble(tokens[i + 1]), (tokens[i + 2] == "") ? double.NaN : Convert.ToDouble(tokens[i + 2]), (tokens[i + 3] == "") ? 0 : Convert.ToInt32(tokens[i + 3])));
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                }

                                if (seqNum == totalNum)
                                {
                                    int sVsInView;
                                    try
                                    {
                                        sVsInView = Convert.ToInt32(tokens[3]) - Gpgsv_errCount;
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                    MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gpgsv(GpgsvRaw, sVsInView, Gpgsv_sat)));
                                    Gpgsv_sat = new List<SatelliteVehicle>();
                                }
                            }
                            break;

                        case "GLGSV":
                            {
                                if (tokens.Length < 4)
                                    break;

                                if (tokens.Length % 4 == 1)
                                    tokens = tokens.Take(tokens.Length - 1).ToArray();


                                int seqNum;
                                try
                                {
                                    seqNum = Convert.ToInt32(tokens[2]);
                                }
                                catch
                                {
                                    throw;
                                }
                                if (seqNum != Glgsv_count)
                                {
                                    break;
                                }

                                int totalNum;
                                try
                                {
                                    totalNum = Convert.ToInt32(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                for (int i = 4; i < tokens.Length; i += 4)
                                {
                                    try
                                    {
                                        if (tokens[i] == "")
                                        {
                                            if (tokens[i + 3] != "")
                                            {
                                                Glgsv_errCount++;
                                            }
                                            continue;
                                        }

                                        Glgsv_sat.Add(new SatelliteVehicle(Convert.ToInt32(tokens[i]), (tokens[i + 1] == "") ? double.NaN : Convert.ToDouble(tokens[i + 1]), (tokens[i + 2] == "") ? double.NaN : Convert.ToDouble(tokens[i + 2]), (tokens[i + 3] == "") ? 0 : Convert.ToInt32(tokens[i + 3])));
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                }

                                if (seqNum == totalNum)
                                {
                                    int sVsInView;
                                    try
                                    {
                                        sVsInView = Convert.ToInt32(tokens[3]) - Glgsv_errCount;
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                    MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Glgsv(GlgsvRaw, sVsInView, Glgsv_sat)));
                                    Glgsv_sat = new List<SatelliteVehicle>();
                                }
                            }
                            break;

                        case "GAGSV": //Galileo
                            {
                                if (tokens.Length < 4)
                                    break;

                                if (tokens.Length % 4 == 1)
                                    tokens = tokens.Take(tokens.Length - 1).ToArray();

                                int seqNum;
                                try
                                {
                                    seqNum = Convert.ToInt32(tokens[2]);
                                }
                                catch
                                {
                                    throw;
                                }
                                if (seqNum != Gagsv_count)
                                {
                                    break;
                                }

                                int totalNum;
                                try
                                {
                                    totalNum = Convert.ToInt32(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                for (int i = 4; i < tokens.Length; i += 4)
                                {
                                    try
                                    {
                                        if (tokens[i] == "")
                                        {
                                            if (tokens[i + 3] != "")
                                            {
                                                Gagsv_errCount++;
                                            }
                                            continue;
                                        }

                                        Gagsv_sat.Add(new SatelliteVehicle(Convert.ToInt32(tokens[i]), (tokens[i + 1] == "") ? double.NaN : Convert.ToDouble(tokens[i + 1]), (tokens[i + 2] == "") ? double.NaN : Convert.ToDouble(tokens[i + 2]), (tokens[i + 3] == "") ? 0 : Convert.ToInt32(tokens[i + 3])));
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                }

                                if (seqNum == totalNum)
                                {
                                    int sVsInView;
                                    try
                                    {
                                        sVsInView = Convert.ToInt32(tokens[3]) - Gagsv_errCount;
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                    MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Gagsv(GagsvRaw, sVsInView, Gagsv_sat)));
                                    Gagsv_sat = new List<SatelliteVehicle>();
                                }
                            }
                            break;

                        case "REFCHECK":
                            {
                                if (tokens.Length != 10)
                                    break;

                                PpsStatus status;
                                switch (tokens[9])
                                {
                                    case "OK":
                                        status = PpsStatus.OK;
                                        break;

                                    case "???":
                                        status = PpsStatus.notPossible;
                                        break;

                                    case "GLITCH":
                                        status = PpsStatus.glitch;
                                        break;

                                    case "OLD":
                                        status = PpsStatus.old;
                                        break;

                                    case "CHECK_MISSING":
                                        status = PpsStatus.ppsMissing;
                                        break;

                                    case "REF_MISSING":
                                        status = PpsStatus.refMissing;
                                        break;

                                    default:
                                        status = PpsStatus.unexpected;
                                        break;
                                }

                                double softwareDelay;
                                try
                                {
                                    softwareDelay = Convert.ToDouble(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double skew;
                                try
                                {
                                    skew = Convert.ToDouble(tokens[3]);
                                }
                                catch
                                {
                                    throw;
                                }

                                Int64 absTime_ref;
                                try
                                {
                                    if (status == PpsStatus.glitch || status == PpsStatus.ppsMissing || status == PpsStatus.refMissing)
                                    {
                                        absTime_ref = -1;
                                    }
                                    else
                                    {
                                        absTime_ref = Convert.ToInt64(tokens[5]);
                                    }
                                }
                                catch
                                {
                                    throw;
                                }

                                Int64 absTime_check;
                                try
                                {
                                    if (status == PpsStatus.glitch || status == PpsStatus.ppsMissing)
                                    {
                                        absTime_check = -1;
                                    }
                                    else
                                    {
                                        absTime_check = Convert.ToInt64(tokens[7]);
                                    }
                                }
                                catch
                                {
                                    throw;
                                }

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Refcheck(raw, softwareDelay, skew, absTime_ref, absTime_check, status)));
                            }
                            break;

                        case "REFTIME":
                            {
                                if (tokens.Length != 3)
                                    break;

                                Int64 absTime;
                                try
                                {
                                    absTime = Convert.ToInt64(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Reftime(raw, absTime)));
                            }
                            break;

                        case "EXTRAREF":
                            {
                                if (tokens.Length != 2)
                                    break;

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Extraref(raw, tokens[1])));
                            }
                            break;

                        case "SKEW":
                            {
                                if (tokens.Length != 8)
                                    break;

                                PpsStatus status;
                                switch (tokens[7])
                                {
                                    case "OK":
                                        status = PpsStatus.OK;
                                        break;

                                    case "???":
                                        status = PpsStatus.notPossible;
                                        break;

                                    case "GLITCH":
                                        status = PpsStatus.glitch;
                                        break;

                                    case "OLD":
                                        status = PpsStatus.old;
                                        break;

                                    case "PPS_MISSING":
                                        status = PpsStatus.ppsMissing;
                                        break;

                                    case "REF_MISSING":
                                        status = PpsStatus.refMissing;
                                        break;

                                    default:
                                        status = PpsStatus.unexpected;
                                        break;
                                }

                                double packetDelay;
                                try
                                {
                                    packetDelay = Convert.ToDouble(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double skew;
                                try
                                {
                                    skew = Convert.ToDouble(tokens[3]);
                                }
                                catch
                                {
                                    throw;
                                }

                                Int64 absTime;
                                try
                                {
                                    if (status == PpsStatus.glitch || status == PpsStatus.ppsMissing)
                                    {
                                        absTime = -1;
                                    }
                                    else
                                    {
                                        absTime = Convert.ToInt64(tokens[5]);
                                    }
                                }
                                catch
                                {
                                    throw;
                                }

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Skew(raw, packetDelay, skew, absTime, status)));
                            }
                            break;

                        case "SENDAT":
                            {
                                if (tokens.Length != 16)
                                    break;

                                SenDataStatus status;
                                switch (tokens[15])
                                {
                                    case "OK":
                                        status = SenDataStatus.OK;
                                        break;

                                    case "OLD":
                                        status = SenDataStatus.old;
                                        break;

                                    default:
                                        status = SenDataStatus.invalid;
                                        break;
                                }

                                double temperature_bmp180;
                                try
                                {
                                    temperature_bmp180 = Convert.ToDouble(tokens[1]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double pressure;
                                try
                                {
                                    pressure = Convert.ToDouble(tokens[3]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double visibleLightIntensity;
                                try
                                {
                                    visibleLightIntensity = Convert.ToDouble(tokens[5]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double infraredLighIntensity;
                                try
                                {
                                    infraredLighIntensity = Convert.ToDouble(tokens[7]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double temperature_sht21;
                                try
                                {
                                    temperature_sht21 = Convert.ToDouble(tokens[9]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double relativeHumidity;
                                try
                                {
                                    relativeHumidity = Convert.ToDouble(tokens[11]);
                                }
                                catch
                                {
                                    throw;
                                }

                                double sampleAge;
                                try
                                {
                                    sampleAge = Convert.ToDouble(tokens[13]);
                                }
                                catch
                                {
                                    throw;
                                }

                                MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new SenData(raw, temperature_bmp180, pressure, visibleLightIntensity, infraredLighIntensity, temperature_sht21, relativeHumidity, sampleAge, status)));
                            }
                            break;

                        case "GPINF":
                            {
                                if (tokens.Length != 6)
                                    break;

                                bool OK = true;

                                double delay = double.NaN;
                                try
                                {
                                    delay = Convert.ToDouble(tokens[1].Substring("DELAY=".Length));
                                }
                                catch
                                {
                                    OK = false;
                                }

                                double delay_A90 = double.NaN;
                                if (OK)
                                    try
                                    {
                                        delay_A90 = Convert.ToDouble(tokens[2].Substring("A90=".Length));
                                    }
                                    catch
                                    {
                                        OK = false;
                                    }

                                double delay_A500 = double.NaN;
                                if (OK)
                                    try
                                    {
                                        delay_A500 = Convert.ToDouble(tokens[3].Substring("A500=".Length));
                                    }
                                    catch
                                    {
                                        OK = false;
                                    }

                                double delay_A1000 = double.NaN;
                                if (OK)
                                    try
                                    {
                                        delay_A1000 = Convert.ToDouble(tokens[4].Substring("A1000=".Length));
                                    }
                                    catch
                                    {
                                        OK = false;
                                    }

                                double temperature = double.NaN;
                                if (OK)
                                    try
                                    {
                                        temperature = Convert.ToDouble(tokens[5].Substring("TEMPERATURE=".Length));
                                    }
                                    catch
                                    {
                                        OK = false;
                                    }

                                if (OK)
                                    MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new PpsInfo(raw, delay, delay_A90, delay_A500, delay_A1000, temperature)));

                            }
                            break;

                        default:
                            MessageReceived?.Invoke(this, new NmeaMessageReceivedEventArgs(new Unknown(raw)));
                            break;
                    }

                }
            }
        }
        public void startLogging()
        {
#if !NMEADevice_UnitTest
            logger.Start();
#endif
        }

        CancellationTokenSource isCloseRequested = new CancellationTokenSource();
        public void Close()
        {
            isCloseRequested.Cancel();
            collectorStopped.WaitOne();
#if !NMEADevice_UnitTest
            logger.Close();
#endif
        }

        public void ResetInputStream()
        {
            ResetRequested.Set();
        }
    }
}
