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
        double windowWidth = 500.0;

        // Giữ nguyên các biến này để đảm bảo gửi đủ dữ liệu
        double kp, ki, gammap, gammai, setpoint, ampMax, ampMin;

        bool isOn = false;
        string message = "";

        public Form1()
        {
            InitializeComponent();
            SetupGraph();

            // Mặc định chọn Setpoint
            if (cbInputType.Items.Count > 0) cbInputType.SelectedIndex = 0;

            // Mặc định chọn PID (index 0)
            if (cbControllerType.Items.Count > 0) cbControllerType.SelectedIndex = 0;

            serial.DataReceived += Serial_DataReceived;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // --- PHẦN 1: ẨN GIAO DIỆN MRAS ---
            // Code này sẽ làm biến mất các ô không cần thiết khi chạy lên
            if (cbControllerType != null) cbControllerType.Visible = false; // Ẩn nút chọn Controller
            if (txtGammap != null) txtGammap.Visible = false;               // Ẩn Gamma P
            if (txtGammai != null) txtGammai.Visible = false;               // Ẩn Gamma I
            if (txtref != null) txtref.Visible = false;                     // Ẩn Ref output

            // Lưu ý: Các Label (chữ "Gamma P", "Gamma I"...) bạn nên xóa bằng tay trong Design 
            // hoặc để kệ nó cũng được, code này chỉ ẩn ô nhập liệu.
            // ---------------------------------

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

            pane1.CurveList.Clear();
            pane1.GraphObjList.Clear();

            curveYm = pane1.AddCurve("Setpoint (ym)", listYm, Color.Blue, SymbolType.None);
            curveY = pane1.AddCurve("Actual Speed (y)", listY, Color.Red, SymbolType.None);
            curveYm.Line.Width = 2.0f; // Độ dày cho đường Setpoint (Màu xanh)
            curveY.Line.Width = 2.5f;  // Độ dày cho đường Tốc độ thực (Màu đỏ) - Cho đậm hơn chút để dễ nhìn

            // --- CẤU HÌNH TRỤC Y ---
            pane1.YAxis.Scale.Min = -350;
            pane1.YAxis.Scale.Max = 350;

            LineObj zeroLine = new LineObj(Color.Black, 0, 0, 1, 0);
            zeroLine.Location.CoordinateFrame = CoordType.XChartFractionYScale;
            pane1.GraphObjList.Add(zeroLine);

            // Setup biểu đồ 2 (Control)
            GraphPane pane2 = zedGraph2.GraphPane;
            pane2.CurveList.Clear();
            curveUc = pane2.AddCurve("Control (u)", listUc, Color.Blue, SymbolType.None);
            curveUc.Line.Width = 2.0f; // Độ dày cho đường Setpoint (Màu xanh)
            pane2.YAxis.Scale.Min = 0;
            pane2.YAxis.Scale.Max = 100;

            zedGraph1.AxisChange();
            zedGraph2.AxisChange();
        }

        private void btn_Connect_Click(object sender, EventArgs e)
        {
            try
            {
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
            isOn = !isOn;
        }

        //private void Serial_DataReceived(object sender, SerialDataReceivedEventArgs e)
        //{
        //    if (!serial.IsOpen) return;
        //    try
        //    {
        //        string line = serial.ReadLine();
        //        if (string.IsNullOrEmpty(line)) return;

        //        this.BeginInvoke(new Action(() =>
        //        {
        //            try
        //            {
        //                string[] parts = line.Split(',');

        //                if (parts.Length >= 3)
        //                {
        //                    double rawYm, rawY, rawUc;

        //                    double.TryParse(parts[0], out rawYm);
        //                    double.TryParse(parts[1], out rawY);
        //                    double.TryParse(parts[2], out rawUc);

        //                    ym = rawYm;

        //                    // Logic xử lý dấu
        //                    if (ym < 0) y = -Math.Abs(rawY);
        //                    else y = Math.Abs(rawY);

        //                    uc = rawUc;

        //                    listYm.Add(time, ym);
        //                    listY.Add(time, y);
        //                    listUc.Add(time, uc);
        //                    time += 1;

        //                    // --- PHẦN 2: CHỈNH HIỂN THỊ ---
        //                    // Luôn ép kiểu hiển thị là PID bình thường (không dùng isMRAS nữa)
        //                    bool isMRAS = false;

        //                    if (isMRAS)
        //                    {
        //                        // Đoạn này sẽ không bao giờ chạy vào
        //                        if (txtref != null) txtref.Text = ym.ToString("0.0");
        //                    }
        //                    else if (!isEditingSetpoint)
        //                    {
        //                        txtSetpoint.Text = ym.ToString("0.0");
        //                    }

        //                    txtplant.Text = y.ToString("0.0");

        //                    zedGraph1.AxisChange();
        //                    zedGraph1.Invalidate();

        //                    zedGraph2.AxisChange();
        //                    zedGraph2.Invalidate();
        //                }
        //            }
        //            catch (Exception ex)
        //            {
        //                // Bỏ qua lỗi parse
        //            }
        //        }));
        //    }
        //    catch (Exception)
        //    {
        //        // Bỏ qua lỗi Serial
        //    }
        //}
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
                        string[] parts = line.Split(',');

                        if (parts.Length >= 3)
                        {
                            double rawYm, rawY, rawUc;

                            double.TryParse(parts[0], out rawYm);
                            double.TryParse(parts[1], out rawY);
                            double.TryParse(parts[2], out rawUc);

                            ym = rawYm;
                            // Logic xử lý dấu
                            if (ym < 0) y = -Math.Abs(rawY);
                            else y = Math.Abs(rawY);

                            uc = rawUc;

                            // Thêm dữ liệu vào list
                            listYm.Add(time, ym);
                            listY.Add(time, y);
                            listUc.Add(time, uc);

                            // --- ĐÂY LÀ PHẦN XỬ LÝ SCROLLING (REAL-TIME) ---

                            // Lấy đối tượng trục X của biểu đồ 1
                            Scale xScale1 = zedGraph1.GraphPane.XAxis.Scale;

                            // Nếu thời gian chạy vượt quá độ rộng cửa sổ (ví dụ > 60)
                            if (time > windowWidth)
                            {
                                // Dịch chuyển trục X: Min = hiện tại - cửa sổ; Max = hiện tại
                                xScale1.Min = time - windowWidth;
                                xScale1.Max = time;
                            }
                            else
                            {
                                // Nếu chưa đầy màn hình thì giữ nguyên từ 0 đến độ rộng cửa sổ
                                xScale1.Min = 0;
                                xScale1.Max = windowWidth;
                            }

                            // Làm tương tự cho biểu đồ 2 (Control) để 2 biểu đồ chạy đồng bộ
                            Scale xScale2 = zedGraph2.GraphPane.XAxis.Scale;
                            xScale2.Min = xScale1.Min;
                            xScale2.Max = xScale1.Max;

                            // ------------------------------------------------

                            time += 1; // Tăng trục thời gian (sample)

                            // Cập nhật TextBox
                            //if (!isEditingSetpoint)
                            //    txtSetpoint.Text = ym.ToString("0.0");
                            txtplant.Text = y.ToString("0.0");

                            // Vẽ lại đồ thị
                            zedGraph1.AxisChange();
                            zedGraph1.Invalidate();

                            zedGraph2.AxisChange();
                            zedGraph2.Invalidate();
                        }
                    }
                    catch (Exception ex)
                    {
                        // Bỏ qua lỗi parse
                    }
                }));
            }
            catch (Exception)
            {
                // Bỏ qua lỗi Serial
            }
        }

        private void btnSet_Click(object sender, EventArgs e)
        {
            try
            {
                if (isOn) message += $"0";
                else message += $"1";

                // --- PHẦN 3: HARDCODE DỮ LIỆU GỬI ĐI ---
                // Thay vì lấy từ GUI (đã bị ẩn), ta gán cứng giá trị để gửi cho MCU đúng format

                // 1. Controller Type: Luôn gửi 0 (PID)
                int controllerType = 0;

                int inputType = cbInputType.Text == "Setpoint" ? 0 :
                                cbInputType.Text == "Sine" ? 1 :
                                2; // Pulse

                double.TryParse(txtKp.Text, out kp);
                double.TryParse(txtKi.Text, out ki);

                // 2. Gamma: Luôn gửi 0 (Vì đang chạy PID)
                gammap = 0;
                gammai = 0;

                double.TryParse(txtAmpMin.Text, out ampMin);
                double.TryParse(txtAmpMax.Text, out ampMax);
                double.TryParse(txtSetpoint.Text, out setpoint);

                if (ampMax < ampMin)
                {
                    MessageBox.Show("Lỗi nhập giá trị ampMax và ampMin");
                    return;
                }

                // Chuỗi gửi đi vẫn GIỮ NGUYÊN cấu trúc cũ
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
            // Hàm này bây giờ vô hiệu hóa vì ta đã ẩn ComboBox rồi
            // Tuy nhiên vẫn để code xử lý logic enable/disable phòng hờ
            // Nhưng ép về logic PID

            bool isMRAS = false; // Luôn coi là False

            if (txtGammap != null) txtGammap.Enabled = isMRAS;
            if (txtGammai != null) txtGammai.Enabled = isMRAS;

            txtKp.Enabled = txtKi.Enabled = !isMRAS;
            if (txtref != null) txtref.Enabled = isMRAS;

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