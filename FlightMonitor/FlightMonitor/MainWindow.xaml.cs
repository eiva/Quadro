using System;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows;

using UsbHidLib;

namespace FlightMonitor
{
   [StructLayout(LayoutKind.Sequential, Size = 6, Pack = 1)]
   struct Data3
   {
      public UInt16 X, Y, Z;

      public override string ToString()
      {
         return String.Format("[{0,10}] [{1,10}] [{2,10}]", X, Y, Z);
      }
   }

   [StructLayout(LayoutKind.Sequential, Size = 8, Pack = 1)]
   struct Data4
   {
      public UInt16 D1, D2, D3, D4;
      public override string ToString()
      {
         return String.Format("[{0,10}] [{1,10}] [{2,10}] [{3,10}]", D1, D2, D3, D4);
      }
   }

   [StructLayout(LayoutKind.Sequential, Pack = 1)]
   struct Packet
   {
      public Data3 SensorAccelerometer;
      public Data3 SensorGyro;
      public Data3 SensorMagnetometer;
      public Data4 RadioData;
      public Data4 MotorGear;
      public UInt16 DeltaMicroseconds;

      public override string ToString()
      {
         var sb = new StringBuilder();
         sb.Append("SA: ");
         sb.AppendLine(SensorAccelerometer.ToString());
         sb.Append("SG: ");
         sb.AppendLine(SensorGyro.ToString());
         sb.Append("SM: ");
         sb.AppendLine(SensorMagnetometer.ToString());
         sb.Append("IR: ");
         sb.AppendLine(RadioData.ToString());
         sb.Append("MD: ");
         sb.AppendLine(MotorGear.ToString());
         sb.Append("DT: ");
         sb.Append(DeltaMicroseconds);
         return sb.ToString();
      }
   }

   internal static class DataHelpers
   {
      public static T ByteArrayToStructure<T>(byte[] bytes) where T : struct
      {
         var type = typeof (T);
         int structSize = Marshal.SizeOf(type);
         var buff = new byte[structSize];

         Array.Copy(bytes, 0, buff, 0, Math.Min(bytes.Length, structSize));

         var handle = GCHandle.Alloc(buff, GCHandleType.Pinned);
         var stuff = (T)Marshal.PtrToStructure(handle.AddrOfPinnedObject(), type);
         handle.Free();
         return stuff;
      }

      public static T[] SubArray<T>(this T[] data, int index, int length)
      {
         T[] result = new T[length];
         Array.Copy(data, index, result, 0, length);
         return result;
      }
   }

   /// <summary>
   /// Interaction logic for MainWindow.xaml
   /// </summary>
   public partial class MainWindow
   {
      private Device _device;

      private volatile bool _isUpdateInProgress;

      public MainWindow()
      {
         InitializeComponent();
      }

      private void Button_Click(object sender, RoutedEventArgs e)
      {
         if (_device != null)
         {
            _device.Dispose();
            _device = null;
         }
         var devices = Browser.Browse();
         _log.Text = String.Join("\n", devices.Select(info => String.Format("\"{0}\" VID={1:X4}, PID={2:X4}", info.Product, info.Vid, info.Pid)));
         var stm = devices.FirstOrDefault(info => info.Product.Contains("STM"));
         if (stm == null)
         {
            return;
         }
         _device = new Device(stm.Path);
         _device.StartReading(data =>
                              {
                                 if (_isUpdateInProgress) return; // Prevent overspamming.
                                 _isUpdateInProgress = true;
                                 Application.Current.Dispatcher.BeginInvoke(new Action(() => dataArrived(data)));
                              });
      }
      
      private void dataArrived(byte[] data)
      {
         var packet = DataHelpers.ByteArrayToStructure<Packet>(data.SubArray(1, data.Length-1));
         _readData.Text =String.Join(" ", data.Select(b=>b.ToString("X2"))) + Environment.NewLine + packet;
         _isUpdateInProgress = false;
      }
   }
}
