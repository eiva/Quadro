using System;
using System.Linq;
using System.Windows;

using UsbHidLib;

namespace FlightMonitor
{
   /// <summary>
   /// Interaction logic for MainWindow.xaml
   /// </summary>
   public partial class MainWindow
   {
      private Device _device;
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
         _device.StartReading(data => Application.Current.Dispatcher.BeginInvoke(new Action(()=>dataArrived(data))));
      }

      private void dataArrived(byte[] data)
      {
         _readData.Text = String.Join(":", data.Select(b=>b.ToString("X2")));
      }
   }
}
