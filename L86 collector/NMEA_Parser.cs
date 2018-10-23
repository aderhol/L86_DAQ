﻿using System;
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
        private Thread SerialCollector;
        private Thread SerialProcessor;

        private ThreadedLogger logger;

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

            SerialCollector = new Thread(new ThreadStart(SerialCollectorWorker));
            SerialCollector.Name = "SerialCollector: " + name;
            SerialCollector.IsBackground = true;

            SerialProcessor = new Thread(new ThreadStart(Process));
            SerialProcessor.Name = "SeralProcessor: " + name;
            SerialProcessor.IsBackground = true;
        }

        public void OpenPort()
        {
            if (Port.IsOpen)
                throw new Exception("NMEA_Parser: double port open");

            SerialCollector.Start();
            SerialProcessor.Start();
        }

        private void SerialCollectorWorker()
        {
            Port.ReadBufferSize = 51200;
            Port.Open();

            int contCount = 0;
            while (true)
            {
                Thread.Sleep(100);
                if (Port.BytesToRead < 30 && (contCount < 5 || Port.BytesToRead == 0))
                {
                    contCount++;
                    continue;
                }

                contCount = 0;
                string snippet = Port.ReadExisting();
                logger.Log(snippet);
                IncomingQueue.Enqueue(snippet);
            }
        }

        private int Gpgsv_count = 0;
        private List<SatelliteVehicle> Gpgsv_sat = new List<SatelliteVehicle>();
        private string GpgsvRaw = "";
        private int Glgsv_count = 0;
        private List<SatelliteVehicle> Glgsv_sat = new List<SatelliteVehicle>();
        private string GlgsvRaw = "";
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

                    int starIndex = raw.IndexOf('*');
                    if (starIndex == -1)
                        continue;   //* was not found
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
                    else
                    {
                        Gpgsv_count = 0;
                        Gpgsv_sat = new List<SatelliteVehicle>();
                        GpgsvRaw = "";
                    }

                    if (tokens[0] == "GLGSV")
                    {
                        GlgsvRaw += ((GlgsvRaw == "") ? ("") : ("\r\n")) + raw;
                        Glgsv_count++;
                    }
                    else
                    {
                        Glgsv_count = 0;
                        Glgsv_sat = new List<SatelliteVehicle>();
                        GlgsvRaw = "";
                    }

                    switch (tokens[0])
                    {
                        case "GPTXT":
                            MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gptxt(raw)));
                            break;
                        case "GNTXT":
                            MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gntxt(raw)));
                            break;

                        case "GPRMC":
                            {
                                if (tokens.Length != 13)
                                    break;


                                DateTime dateTime;
                                try
                                {
                                    dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.fff ddMMyy", null);
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

                                MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gprmc(raw, dateTime, active, latitude, longitude, speed, course)));
                            }
                            break;

                        case "GNRMC":
                            {
                                if (tokens.Length != 13)
                                    break;


                                DateTime dateTime;
                                try
                                {
                                    dateTime = DateTime.ParseExact(tokens[1] + " " + tokens[9], "HHmmss.fff ddMMyy", null);
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

                                MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gnrmc(raw, dateTime, active, latitude, longitude, speed, course)));
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

                                MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gpgga(raw, quality, numberOfSatellites, altitude, timeSinceLastDgpsUpdate, dgpsStationId)));
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

                                MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gngga(raw, quality, numberOfSatellites, altitude, timeSinceLastDgpsUpdate, dgpsStationId)));
                            }
                            break;

                        case "GNGSA":
                            {
                                if (tokens.Length != 18)
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

                                MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gngsa(raw, fixMode, hdop, vdop, pdop, sVs)));
                            }
                            break;

                        case "GPGSA":
                            {
                                if (tokens.Length != 18)
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

                                MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gpgsa(raw, fixMode, hdop, vdop, pdop, sVs)));
                            }
                            break;

                        case "GPGSV":
                            {
                                if (tokens.Length < 4)
                                    break;

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
                                        sVsInView = Convert.ToInt32(tokens[3]);
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                    MessageReceived(this, new NmeaMessageReceivedEventArgs(new Gpgsv(GpgsvRaw, sVsInView, Gpgsv_sat)));
                                    Gpgsv_sat = new List<SatelliteVehicle>();
                                }
                            }
                            break;

                        case "GLGSV":
                            {
                                if (tokens.Length < 4)
                                    break;

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
                                        sVsInView = Convert.ToInt32(tokens[3]);
                                    }
                                    catch
                                    {
                                        throw;
                                    }
                                    MessageReceived(this, new NmeaMessageReceivedEventArgs(new Glgsv(GlgsvRaw, sVsInView, Glgsv_sat)));
                                    Glgsv_sat = new List<SatelliteVehicle>();
                                }
                            }
                            break;

                        default:
                            MessageReceived(this, new NmeaMessageReceivedEventArgs(new Unknown(raw)));
                            break;
                    }

                }
            }
        }
        public void startLogging()
        {
            logger.Start();
        }
    }
}
