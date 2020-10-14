using System;
using System.IO;
using System.Net.Sockets;

namespace MavlinkTestServer
{
    class TcpClientCom : TcpClient
    {
        public const int BUFF_SIZE = 65536;
        NetworkStream serverStream_;
        byte[] bytesIn_;
        //private Action<byte[]> function;
        //readonly MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();

        bool isDead_ = true;

        public bool IsDead { get { return isDead_; } }

        public TcpClientCom()
        {
            bytesIn_ = new byte[BUFF_SIZE];
        }

        public bool ConnectTo(string ip, int port)
        {
            return Connect(new NetworkAddress(ip, port));
        }

        public bool Connect(NetworkAddress srvrAddr)
        {
            try
            {
                if (!srvrAddr.IsIpValid() || !srvrAddr.IsPortValid())
                    return false;
                if (ConnectAsync(srvrAddr.IP.ToString(), srvrAddr.Port).Wait(100))
                {
                    isDead_ = false;
                    serverStream_ = GetStream();
                    AsyncListen();
                }
            }
            catch (SocketException)
            {
                //wrong address
                isDead_ = true;
            }
            return !isDead_;
        }

        public void Subscribe(Action<byte[]> func)
        {
            if (func is null)
            {
                throw new ArgumentNullException(nameof(func));
            }
            //function = func;
        }

        private void AsyncListen()
        {
            serverStream_.BeginRead(bytesIn_, 0, bytesIn_.Length, new AsyncCallback(OnReadCB), null);
        }

        private void OnReadCB(IAsyncResult ar)
        {
            try
            {
                if (IsDead)
                    return;
                int numberOfBytesRead = serverStream_.EndRead(ar);
                if (numberOfBytesRead == 0)
                    return;
                byte[] truncArray = new byte[numberOfBytesRead];
                Array.Copy(bytesIn_, truncArray, truncArray.Length);
                MAVLink.MAVLinkMessage message = new MAVLink.MAVLinkMessage(truncArray);
                //MavlinkHandler.GetInstance().ParsePacket(message);
                bytesIn_ = new byte[BUFF_SIZE];
                AsyncListen();
            }
            catch { }
        }

        public bool Send(byte[] bytes)
        {
            if (!IsDead)
            {
                try
                {
                    serverStream_.BeginWrite(bytes, 0, bytes.Length, new AsyncCallback(OnWriteCB), null);
                }
                catch (IOException)
                {
                    return false;
                }
                return true;
            }
            return false;
        }
        private void OnWriteCB(IAsyncResult ar)
        {
            try
            {
                if (IsDead)
                    return;
                serverStream_.EndWrite(ar);
            }
            catch { }
        }

        public void CloseMe()
        {
            isDead_ = true;
            serverStream_.Close();
            Close();
        }
    }
}
