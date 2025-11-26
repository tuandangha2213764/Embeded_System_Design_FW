using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using ZedGraph;

namespace Motor_control
{
    
    public partial class Form1 : Form
    {
        bool isEditingSetpoint = false;
        SerialPort serial = new SerialPort();
        RollingPointPairList listYm = new RollingPointPairList(1000);
        RollingPointPairList listY = new RollingPointPairList(1000);
        RollingPointPairList listUc = new RollingPointPairList(1000);
        LineItem curveYm, curveY, curveUc;
        double time = 0;
        double ym = 0, y = 0, uc = 0;
        double kp, ki, gammap, gammai, setpoint, ampMax, ampMin;
        bool isOn = false;
        string message = "";

        public Form1()
        {
            InitializeComponent();
            SetupGraph();
            cbInputType.SelectedIndex = 0;
            cbControllerType.SelectedIndex = 0;
            serial.DataReceived += Serial_DataReceived;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            txtSetpoint.Enter += (s, ev) => isEditingSetpoint = true;
            txtSetpoint.Leave += (s, ev) => isEditingSetpoint = false;
            string[] ports = SerialPort.GetPortNames();
            Array.Sort(ports);
            cbPort.Items.AddRange(ports);
            if (cbPort.Items.Count > 0)
                cbPort.SelectedIndex = 0;
            progressBar1.Value = 0;
            lblStatus.Text = "DISCONNECTED";
        }

        private void SetupGraph()
        {
            GraphPane pane1 = zedGraph1.GraphPane;
            pane1.Title.Text = "Đáp ứng tốc độ";
            pane1.XAxis.Title.Text = "Time";
            pane1.YAxis.Title.Text = "Speed (RPM)";

            // Xóa list cũ để tránh lỗi
            pane1.CurveList.Clear();
            pane1.GraphObjList.Clear();

            curveYm = pane1.AddCurve("Setpoint (ym)", listYm, Color.Blue, SymbolType.None);
            curveY = pane1.AddCurve("Actual Speed (y)", listY, Color.Red, SymbolType.None);

            // --- CẤU HÌNH TRỤC Y ĐỐI XỨNG ---
            // Cho phép hiển thị cả phần âm và dương
            pane1.YAxis.Scale.Min = -350;
            pane1.YAxis.Scale.Max = 350;
            // --------------------------------

            // Vẽ đường kẻ ngang ở vị trí 0 để dễ nhìn
            LineObj zeroLine = new LineObj(Color.Black, 0, 0, 1, 0);
            zeroLine.Location.CoordinateFrame = CoordType.XChartFractionYScale;
            pane1.GraphObjList.Add(zeroLine);

            // Setup biểu đồ 2 (Control)
            GraphPane pane2 = zedGraph2.GraphPane;
            pane2.CurveList.Clear();
            curveUc = pane2.AddCurve("Control (u)", listUc, Color.Blue, SymbolType.None);
            pane2.YAxis.Scale.Min = 0;
            pane2.YAxis.Scale.Max = 100;

            zedGraph1.AxisChange();
            zedGraph2.AxisChange();
        }

        private void btn_Connect_Click(object sender, EventArgs e)
        {
            try
            {
                // Nếu đang chưa mở cổng thì tiến hành kết nối
                if (!serial.IsOpen)
                {
                    if (cbPort.SelectedItem == null)
                    {
                        MessageBox.Show("Vui lòng chọn cổng COM.");
                        return;
                    }

                    serial.PortName = cbPort.SelectedItem.ToString();
                    serial.BaudRate = 115200;
                    serial.NewLine = "\r\n";
                    serial.Open();

                    progressBar1.Style = ProgressBarStyle.Continuous;
                    progressBar1.Value = 100;

                    lblStatus.Text = "CONNECTED";
                    lblStatus.ForeColor = Color.Green;
                    lblStatus.BackColor = Color.Transparent;

                    btn_Connect.Text = "Disconnect";
                }
                // Nếu đang mở cổng rồi thì ngắt kết nối
                else
                {
                    serial.Close();

                    progressBar1.Value = 0;
                    lblStatus.Text = "DISCONNECTED";
                    lblStatus.ForeColor = Color.Gray;
                    lblStatus.BackColor = Color.Transparent;

                    btn_Connect.Text = "Connect";
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Connection error: " + ex.Message);
                progressBar1.Value = 0;
                lblStatus.Text = "DISCONNECTED";
                lblStatus.ForeColor = Color.Gray;
                lblStatus.BackColor = Color.Transparent;
                btn_Connect.Text = "Connect";
            }
        }

        private void btnBrake_Click(object sender, EventArgs e)
        {
                if (isOn)
                {
                    btnBrake.BackColor = Color.Lime;
                    btnBrake.Text = "ON";
                }
                else
                {               
                    btnBrake.BackColor = Color.Red;
                    btnBrake.Text = "OFF";
                }
                isOn = !isOn; // Dao trang thai
        }

        private void Serial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (!serial.IsOpen) return;
            try
            {
                string line = serial.ReadLine();
                if (string.IsNullOrEmpty(line)) return;

                this.BeginInvoke(new Action(() =>
                {
                    try
                    {
                        // Tách chuỗi gửi từ MCU: "Setpoint, Speed, Control"
                        // Ví dụ: "-100.00, 98.50, 45.00"
                        string[] parts = line.Split(',');

                        if (parts.Length >= 3)
                        {
                            double rawYm, rawY, rawUc;

                            // 1. Parse dữ liệu thô
                            double.TryParse(parts[0], out rawYm); // Setpoint (có thể âm hoặc dương)
                            double.TryParse(parts[1], out rawY);  // Speed (MCU gửi lên luôn dương)
                            double.TryParse(parts[2], out rawUc); // Control signal

                            // 2. --- LOGIC XỬ LÝ DẤU ---

                            // Gán Setpoint
                            ym = rawYm;

                            // Xử lý dấu cho Tốc độ thực (y):
                            // Logic: Nếu Setpoint (ym) đang Âm -> Ép Tốc độ (y) hiển thị Âm theo.
                            // Cách này đúng cho cả Pulse, Sine, Setpoint tĩnh mà không cần if/else phức tạp.
                            if (ym < 0)
                            {
                                y = -Math.Abs(rawY); // Ép hiển thị số âm
                            }
                            else
                            {
                                y = Math.Abs(rawY);  // Ép hiển thị số dương
                            }

                            // Gán tín hiệu điều khiển
                            uc = rawUc;

                            // 3. Đưa vào danh sách vẽ đồ thị
                            listYm.Add(time, ym);
                            listY.Add(time, y);
                            listUc.Add(time, uc);
                            time += 1; // Tăng trục thời gian (bạn có thể chỉnh step nhỏ hơn nếu muốn mượt)

                            // 4. Cập nhật số liệu ra TextBox (chỉ để nhìn)
                            bool isMRAS = cbControllerType.Text == "PI-MRAS";

                            // Cập nhật ô Setpoint (chỉ khi không phải người dùng đang gõ)
                            if (isMRAS)
                                txtref.Text = ym.ToString("0.0");
                            else if (!isEditingSetpoint)
                                txtSetpoint.Text = ym.ToString("0.0");

                            // Cập nhật ô tốc độ thực
                            txtplant.Text = y.ToString("0.0");

                            // 5. Vẽ lại đồ thị (Refresh)
                            zedGraph1.AxisChange();
                            zedGraph1.Invalidate();

                            zedGraph2.AxisChange();
                            zedGraph2.Invalidate();
                        }
                    }
                    catch (Exception ex)
                    {
                        // Bỏ qua lỗi parse data init
                    }
                }));
            }
            catch (Exception)
            {
                // Bỏ qua lỗi Serial timeout hoặc ngắt kết nối đột ngột
            }
        }

        private void btnSet_Click(object sender, EventArgs e)
        {
                try
                {
                    if (isOn)
                    {
                        message += $"0";
                    }
                    else
                    {
                        message += $"1";
                    }
                    int controllerType = cbControllerType.Text == "PI-MRAS" ? 1 : 0;
                    int inputType = cbInputType.Text == "Setpoint" ? 0 :
                                    cbInputType.Text == "Sine" ? 1 :
                                    2; // Pulse
                    double.TryParse(txtKp.Text, out kp);
                    double.TryParse(txtKi.Text, out ki);
                    double.TryParse(txtGammap.Text, out gammap);
                    double.TryParse(txtGammai.Text, out gammai);
                    double.TryParse(txtAmpMin.Text, out ampMin);
                    double.TryParse(txtAmpMax.Text, out ampMax);
                    double.TryParse(txtSetpoint.Text, out setpoint);
                    if (ampMax<ampMin)
                    {
                        MessageBox.Show("Lỗi nhập giá trị ampMax và ampMin");
                        return;
                    }
                    message += $",{controllerType},{inputType},{kp},{ki},{gammap},{gammai},{ampMax},{ampMin},{setpoint}";

                    message += "R";
                    serial.WriteLine(message);
                    message = "";
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Send error: " + ex.Message);
                }    
        }

        private void cbControllerType_SelectedIndexChanged(object sender, EventArgs e)
        {
            bool isMRAS = cbControllerType.Text == "PI-MRAS";
            txtGammap.Enabled = isMRAS;
            txtGammai.Enabled = isMRAS;
            txtKp.Enabled = txtKi.Enabled = !isMRAS;
            txtref.Enabled = isMRAS;
   
            if (!isMRAS) txtSetpoint.Text = ym.ToString();
            zedGraph1.Invalidate();
        }

        private void cbInputType_SelectedIndexChanged(object sender, EventArgs e)
        {
            string inputType = cbInputType.Text;

            if (inputType == "Setpoint")
            {
                txtSetpoint.Enabled = true;
                txtAmpMin.Enabled = false;
                txtAmpMax.Enabled = false;
            }
            else if (inputType == "Sine")
            {
                txtSetpoint.Enabled = false;
                txtAmpMin.Enabled = false;
                txtAmpMax.Enabled = false;
            }
            else if (inputType == "Pulse")
            {
                txtSetpoint.Enabled = false;
                txtAmpMin.Enabled = true;
                txtAmpMax.Enabled = true;
            }
        }
    }
}
