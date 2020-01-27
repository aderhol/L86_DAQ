using System;
using System.Text;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
using System.Collections.Specialized;
using System.IO;

namespace SerialPort_Win32
{
    #region Win32_API

    /*[StructLayout(LayoutKind.Sequential)]
    internal struct COMMPROP
    {
        internal ushort wPacketLength;
        internal ushort wPacketVersion;
        internal uint dwServiceMask;
        internal uint dwReserved1;
        internal uint dwMaxTxQueue;
        internal uint dwMaxRxQueue;
        internal uint dwMaxBaud;
        internal uint dwProvSubType;
        internal uint dwProvCapabilities;
        internal uint dwSettableParams;
        internal uint dwSettableBaud;
        internal ushort wSettableData;
        internal ushort wSettableStopParity;
        internal uint dwCurrentTxQueue;
        internal uint dwCurrentRxQueue;
        internal uint dwProvSpec1;
        internal uint dwProvSpec2;
        internal ushort wcProvChar;
    }*/

    public enum Parity : byte
    {
        NOPARITY = 0,
        ODDPARITY = 1,
        EVENPARITY = 2,
        MARKPARITY = 3,
        SPACEPARITY = 4

    }

    public enum StopBits : byte
    {
        ONESTOPBIT = 0,
        ONE5STOPBITS = 1,
        TWOSTOPBITS = 2
    }
    #endregion
    public class ComPort
    {
        #region Win32_API
        private enum FileAccess : uint
        {
            GENERIC_ALL = (uint)1 << 28,
            GENERIC_EXECUTE = (uint)1 << 29,
            GENERIC_WRITE = (uint)1 << 30,
            GENERIC_READ = (uint)1 << 31
        }

        private enum FileShare : uint
        {
            NONE = 0,
            FILE_SHARE_DELETE = 0x00000004,
            FILE_SHARE_READ = 0x00000001,
            FILE_SHARE_WRITE = 0x00000002
        }

        private enum FileMode : uint
        {
            CREATE_NEW = 1,
            CREATE_ALWAYS = 2,
            OPEN_EXISTING = 3,
            OPEN_ALWAYS = 4,
            TRUNCATE_EXISTING = 5
        }

        private enum FileAttributes : uint
        {
            FILE_ATTRIBUTE_ARCHIVE = 32,
            FILE_ATTRIBUTE_COMPRESSED = 0x800,
            FILE_ATTRIBUTE_DEVICE = 0x40,
            FILE_ATTRIBUTE_DIRECTORY = 0x10,
            FILE_ATTRIBUTE_ENCRYPTED = 0x4000,
            FILE_ATTRIBUTE_HIDDEN = 2,
            FILE_ATTRIBUTE_INTEGRITY_STREAM = 0x8000,
            FILE_ATTRIBUTE_NORMAL = 0x80,
            FILE_ATTRIBUTE_NOT_CONTENT_INDEXED = 0x2000,
            FILE_ATTRIBUTE_NO_SCRUB_DATA = 0x20000,
            FILE_ATTRIBUTE_OFFLINE = 0x1000,
            FILE_ATTRIBUTE_READONLY = 1,
            FILE_ATTRIBUTE_RECALL_ON_DATA_ACCESS = 0x400000,
            FILE_ATTRIBUTE_RECALL_ON_OPEN = 0x40000,
            FILE_ATTRIBUTE_REPARSE_POINT = 0x400,
            FILE_ATTRIBUTE_SPARSE_FILE = 0x200,
            FILE_ATTRIBUTE_SYSTEM = 4,
            FILE_ATTRIBUTE_TEMPORARY = 0x100,
            FILE_ATTRIBUTE_VIRTUAL = 0x10000
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct Dcb
        {
            internal uint DCBLength;
            internal uint BaudRate;
            private BitVector32 Flags;

            private ushort wReserved;        // not currently used
            internal ushort XonLim;           // transmit XON threshold
            internal ushort XoffLim;          // transmit XOFF threshold             

            internal byte ByteSize;
            internal Parity Parity;
            internal StopBits StopBits;

            internal sbyte XonChar;          // Tx and Rx XON character
            internal sbyte XoffChar;         // Tx and Rx XOFF character
            internal sbyte ErrorChar;        // error replacement character
            internal sbyte EofChar;          // end of input character
            internal sbyte EvtChar;          // received event character
            private ushort wReserved1;       // reserved; do not use     

            private static readonly int fBinary;
            private static readonly int fParity;
            private static readonly int fOutxCtsFlow;
            private static readonly int fOutxDsrFlow;
            private static readonly BitVector32.Section fDtrControl;
            private static readonly int fDsrSensitivity;
            private static readonly int fTXContinueOnXoff;
            private static readonly int fOutX;
            private static readonly int fInX;
            private static readonly int fErrorChar;
            private static readonly int fNull;
            private static readonly BitVector32.Section fRtsControl;
            private static readonly int fAbortOnError;

            static Dcb()
            {
                // Create Boolean Mask
                int previousMask;
                fBinary = BitVector32.CreateMask();
                fParity = BitVector32.CreateMask(fBinary);
                fOutxCtsFlow = BitVector32.CreateMask(fParity);
                fOutxDsrFlow = BitVector32.CreateMask(fOutxCtsFlow);
                previousMask = BitVector32.CreateMask(fOutxDsrFlow);
                previousMask = BitVector32.CreateMask(previousMask);
                fDsrSensitivity = BitVector32.CreateMask(previousMask);
                fTXContinueOnXoff = BitVector32.CreateMask(fDsrSensitivity);
                fOutX = BitVector32.CreateMask(fTXContinueOnXoff);
                fInX = BitVector32.CreateMask(fOutX);
                fErrorChar = BitVector32.CreateMask(fInX);
                fNull = BitVector32.CreateMask(fErrorChar);
                previousMask = BitVector32.CreateMask(fNull);
                previousMask = BitVector32.CreateMask(previousMask);
                fAbortOnError = BitVector32.CreateMask(previousMask);

                // Create section Mask
                BitVector32.Section previousSection;
                previousSection = BitVector32.CreateSection(1);
                previousSection = BitVector32.CreateSection(1, previousSection);
                previousSection = BitVector32.CreateSection(1, previousSection);
                previousSection = BitVector32.CreateSection(1, previousSection);
                fDtrControl = BitVector32.CreateSection(2, previousSection);
                previousSection = BitVector32.CreateSection(1, fDtrControl);
                previousSection = BitVector32.CreateSection(1, previousSection);
                previousSection = BitVector32.CreateSection(1, previousSection);
                previousSection = BitVector32.CreateSection(1, previousSection);
                previousSection = BitVector32.CreateSection(1, previousSection);
                previousSection = BitVector32.CreateSection(1, previousSection);
                fRtsControl = BitVector32.CreateSection(3, previousSection);
                previousSection = BitVector32.CreateSection(1, fRtsControl);
            }

            public int DtrControl
            {
                get { return Flags[fDtrControl]; }
                set { Flags[fDtrControl] = value; }
            }

            public int RtsControl
            {
                get { return Flags[fRtsControl]; }
                set { Flags[fRtsControl] = value; }
            }

            public bool Binary
            {
                get { return Flags[fBinary]; }
                set { Flags[fBinary] = value; }
            }

            public bool CheckParity
            {
                get { return Flags[fParity]; }
                set { Flags[fParity] = value; }
            }

            public bool OutxCtsFlow
            {
                get { return Flags[fOutxCtsFlow]; }
                set { Flags[fOutxCtsFlow] = value; }
            }

            public bool OutxDsrFlow
            {
                get { return Flags[fOutxDsrFlow]; }
                set { Flags[fOutxDsrFlow] = value; }
            }

            public bool DsrSensitivity
            {
                get { return Flags[fDsrSensitivity]; }
                set { Flags[fDsrSensitivity] = value; }
            }

            public bool TxContinueOnXoff
            {
                get { return Flags[fTXContinueOnXoff]; }
                set { Flags[fTXContinueOnXoff] = value; }
            }

            public bool OutX
            {
                get { return Flags[fOutX]; }
                set { Flags[fOutX] = value; }
            }

            public bool InX
            {
                get { return Flags[fInX]; }
                set { Flags[fInX] = value; }
            }

            public bool ReplaceErrorChar
            {
                get { return Flags[fErrorChar]; }
                set { Flags[fErrorChar] = value; }
            }

            public bool Null
            {
                get { return Flags[fNull]; }
                set { Flags[fNull] = value; }
            }

            public bool AbortOnError
            {
                get { return Flags[fAbortOnError]; }
                set { Flags[fAbortOnError] = value; }
            }
        }

        private struct COMMTIMEOUTS
        {
            public UInt32 ReadIntervalTimeout;
            public UInt32 ReadTotalTimeoutMultiplier;
            public UInt32 ReadTotalTimeoutConstant;
            public UInt32 WriteTotalTimeoutMultiplier;
            public UInt32 WriteTotalTimeoutConstant;
        }

        /*[DllImport("kernel32.dll")]
        private static extern bool GetCommProperties(SafeFileHandle hFile, ref COMMPROP lpCommProp);*/

        [DllImport("kernel32.dll")]
        private static extern bool SetCommMask(SafeFileHandle hFile, uint dwEvtMask);

        [DllImport("kernel32.dll")]
        private static extern bool SetupComm(SafeFileHandle hFile, uint dwInQueue, uint dwOutQueue);

        [DllImport("kernel32.dll")]
        private static extern bool PurgeComm(SafeFileHandle hFile, uint dwFlags);

        [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
        private static extern SafeFileHandle CreateFile(
            string lpFileName,
            [MarshalAs(UnmanagedType.U4)] FileAccess dwDesiredAccess,
            [MarshalAs(UnmanagedType.U4)] FileShare dwShareMode,
            IntPtr lpSecurityAttributes,
            [MarshalAs(UnmanagedType.U4)] FileMode dwCreationDisposition,
            [MarshalAs(UnmanagedType.U4)] FileAttributes dwFlagsAndAttributes,
            IntPtr hTemplateFile);

        [DllImport("kernel32", SetLastError = true)]
        private static extern bool ReadFile(
            SafeFileHandle hFile,
            byte[] aBuffer,
            uint cbToRead,
            ref uint cbThatWereRead,
            IntPtr pOverlapped);

        [DllImport("kernel32.dll")]
        private static extern bool GetCommState(SafeFileHandle hFile, ref Dcb lpDCB);

        [DllImport("kernel32.dll")]
        private static extern bool SetCommState(SafeFileHandle hFile, [In] ref Dcb lpDCB);

        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool SetCommTimeouts(SafeFileHandle hFile, [In] ref COMMTIMEOUTS lpCommTimeouts);

        [DllImport("kernel32.dll")]
        private static extern bool WriteFile(SafeFileHandle hFile,
            byte[] lpBuffer,
            uint nNumberOfBytesToWrite,
            out uint lpNumberOfBytesWritten,
            IntPtr pOverlapped);
        #endregion

        private SafeFileHandle handle;
        private readonly COMMTIMEOUTS timeouts;

        private int portNum;
        private int baudRate;
        private byte dataBits;
        private Parity parity;
        private StopBits stopBits;
        private uint driverInputBufferSize_bytes;
        private uint driverOutputBufferSize_bytes;

        public int BaudRate
        {
            get
            {
                return baudRate;
            }

            set
            {
                baudRate = value;

                if (IsOpen)
                {
                    refreshPortConfig();
                }
            }
        }

        public byte DataBits
        {
            get
            {
                return dataBits;
            }

            set
            {
                dataBits = value;

                if (IsOpen)
                {
                    refreshPortConfig();
                }
            }
        }

        public Parity Parity
        {
            get
            {
                return parity;
            }

            set
            {
                parity = value;

                if (IsOpen)
                {
                    refreshPortConfig();
                }
            }
        }

        public StopBits StopBits
        {
            get
            {
                return stopBits;
            }

            set
            {
                stopBits = value;

                if (IsOpen)
                {
                    refreshPortConfig();
                }
            }
        }

        public uint InputBufferSize_bytes
        {
            get
            {
                return driverInputBufferSize_bytes;
            }

            set
            {
                driverInputBufferSize_bytes = value;

                if (IsOpen)
                {
                    refreshPortConfig();
                }
            }
        }

        public uint OutputBufferSize_bytes
        {
            get
            {
                return driverOutputBufferSize_bytes;
            }

            set
            {
                driverOutputBufferSize_bytes = value;

                if (IsOpen)
                {
                    refreshPortConfig();
                }
            }
        }


        public bool IsOpen { get; private set; }

        public ComPort(int portNum)
        {
            timeouts.ReadIntervalTimeout = uint.MaxValue;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;

            this.portNum = portNum;

            baudRate = 115200;
            dataBits = 8;
            parity = Parity.NOPARITY;
            stopBits = StopBits.ONESTOPBIT;

            driverInputBufferSize_bytes = 1024;
            driverOutputBufferSize_bytes = 1024;

            IsOpen = false;
        }

        public ComPort(int portNum, int baudRate)
        {
            timeouts.ReadIntervalTimeout = uint.MaxValue;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;

            this.portNum = portNum;

            this.baudRate = baudRate;
            dataBits = 8;
            parity = Parity.NOPARITY;
            stopBits = StopBits.ONESTOPBIT;

            driverInputBufferSize_bytes = 1024;
            driverOutputBufferSize_bytes = 1024;

            IsOpen = false;
        }

        public ComPort(int portNum, int baudRate, Parity parity)
        {
            timeouts.ReadIntervalTimeout = uint.MaxValue;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;

            this.portNum = portNum;

            this.baudRate = baudRate;
            dataBits = 8;
            this.parity = parity;
            stopBits = StopBits.ONESTOPBIT;

            driverInputBufferSize_bytes = 1024;
            driverOutputBufferSize_bytes = 1024;

            IsOpen = false;
        }

        public ComPort(int portNum, int baudRate, Parity parity, byte dataBits)
        {
            timeouts.ReadIntervalTimeout = uint.MaxValue;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;

            this.portNum = portNum;

            this.baudRate = baudRate;
            this.dataBits = dataBits;
            this.parity = parity;
            stopBits = StopBits.ONESTOPBIT;

            driverInputBufferSize_bytes = 1024;
            driverOutputBufferSize_bytes = 1024;

            IsOpen = false;
        }

        public ComPort(int portNum, int baudRate, Parity parity, byte dataBits, StopBits stopBits)
        {
            timeouts.ReadIntervalTimeout = uint.MaxValue;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;

            this.portNum = portNum;

            this.baudRate = baudRate;
            this.dataBits = dataBits;
            this.parity = parity;
            this.stopBits = stopBits;

            driverInputBufferSize_bytes = 1024;
            driverOutputBufferSize_bytes = 1024;

            IsOpen = false;
        }
        private const uint PURGE_TXABORT = 0x0001; // Kill the pending/current writes to the comm port.
        private const uint PURGE_RXABORT = 0x0002; // Kill the pending/current reads to the comm port.
        private const uint PURGE_TXCLEAR = 0x0004; // Kill the transmit queue if there.
        private const uint PURGE_RXCLEAR = 0x0008; // Kill the typeahead buffer if there.
        public void Open()
        {
            if (!IsOpen)
            {
                handle = CreateFile(@"\\.\COM" + portNum,          // for COM1—COM9 only
                    FileAccess.GENERIC_READ | FileAccess.GENERIC_WRITE, //Read/Write
                    FileShare.NONE,               // No Sharing
                    IntPtr.Zero,            // No Security
                    FileMode.OPEN_EXISTING,   // Open existing port only
                    0,               // Non Overlapped I/O
                    IntPtr.Zero);
                int errorCode = Marshal.GetLastWin32Error();
                if (0 != errorCode)
                {
                    throw new IOException("Win32 API CreateFile() failed", errorCode);
                }

                bool OK;

                OK = SetCommMask(handle, 0);
                if (!OK)
                {
                    throw new IOException("Win32 API SetCommMask() failed", Marshal.GetLastWin32Error());
                }

                COMMTIMEOUTS comTimeouts = timeouts;
                OK = SetCommTimeouts(handle, ref comTimeouts);
                if (!OK)
                {
                    throw new IOException("Win32 API SetCommTimeouts() failed", Marshal.GetLastWin32Error());
                }

                refreshPortConfig();

                OK = PurgeComm(handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
                if (!OK)
                {
                    throw new IOException("Win32 API PurgeComm() failed", Marshal.GetLastWin32Error());
                }

                IsOpen = true;
            }
        }

        public void Close()
        {
            if (IsOpen)
            {
                handle.Close();

                IsOpen = false;
            }
        }

        private const int bufferSize_bytes = 100;
        public string ReadExisting()
        {
            if (!IsOpen)
            {
                throw new IOException("The COM port is not open.");
            }

            StringBuilder str = new StringBuilder();

            byte[] buff = new byte[bufferSize_bytes];
            uint cnt = 0;

            bool OK;

            //this is problematic due to possibly becomming an infinte loop, beacuse chars keeo arriving, while it tries to empty the COM ports buffer
            do
            {
                OK = ReadFile(handle, buff, (uint)buff.Length, ref cnt, IntPtr.Zero);
                if (!OK)
                {
                    throw new IOException("Win32 API ReadFile() failed", Marshal.GetLastWin32Error());
                }

                if (cnt > 0)
                {
                    str.Append(Encoding.ASCII.GetString(buff, 0, (int)cnt));
                }
            } while (cnt > 0);

            return str.ToString();
        }

        public void Write(string value)
        {
            if (!IsOpen)
            {
                throw new IOException("The COM port is not open.");
            }

            bool OK;

            byte[] str = Encoding.ASCII.GetBytes(value);

            uint cnt = 0;

            OK = WriteFile(handle, str, (uint)str.Length, out cnt, IntPtr.Zero);
            if (!OK)
            {
                throw new IOException("Win32 API WriteFile() failed", Marshal.GetLastWin32Error());
            }

            if (str.Length != cnt)
            {
                throw new IOException(String.Format("Win32 API WriteFile() error: partial write ({0} out of {1})", cnt, str.Length));
            }
        }

        public void WriteLine(string value)
        {
            Write(value + "\r\n");
        }

        private void refreshPortConfig()
        {
            Dcb dcb = new Dcb();
            dcb.DCBLength = (uint)Marshal.SizeOf(typeof(Dcb));

            bool OK;

            OK = GetCommState(handle, ref dcb);
            if (!OK)
            {
                throw new IOException("Win32 API GetCommState() failed", Marshal.GetLastWin32Error());
            }

            dcb.BaudRate = (uint)baudRate;
            dcb.ByteSize = dataBits;
            dcb.Parity = parity;
            dcb.StopBits = stopBits;

            dcb.CheckParity = false;
            dcb.OutxCtsFlow = false;
            dcb.OutxDsrFlow = false;
            dcb.DtrControl = 0x00;
            dcb.DsrSensitivity = false;
            dcb.OutX = false;
            dcb.InX = false;
            dcb.ReplaceErrorChar = true;
            dcb.Null = false;
            dcb.RtsControl = 0x00;
            dcb.AbortOnError = false;

            OK = SetCommState(handle, ref dcb);
            if (!OK)
            {
                throw new IOException("Win32 API SetCommState() failed", Marshal.GetLastWin32Error());
            }

            OK = SetupComm(handle, driverInputBufferSize_bytes, driverOutputBufferSize_bytes);
            if (!OK)
            {
                throw new IOException("Win32 API SetupComm() failed", Marshal.GetLastWin32Error());
            }
        }
    }
}
