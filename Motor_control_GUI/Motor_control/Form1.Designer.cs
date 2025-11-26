namespace Motor_control
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.zedGraph1 = new ZedGraph.ZedGraphControl();
            this.zedGraph2 = new ZedGraph.ZedGraphControl();
            this.btnSet = new System.Windows.Forms.Button();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.btn_Connect = new System.Windows.Forms.Button();
            this.txtSetpoint = new System.Windows.Forms.TextBox();
            this.textBox22 = new System.Windows.Forms.TextBox();
            this.textBox18 = new System.Windows.Forms.TextBox();
            this.txtplant = new System.Windows.Forms.TextBox();
            this.txtref = new System.Windows.Forms.TextBox();
            this.textBox13 = new System.Windows.Forms.TextBox();
            this.textBox15 = new System.Windows.Forms.TextBox();
            this.textBox16 = new System.Windows.Forms.TextBox();
            this.cbControllerType = new System.Windows.Forms.ComboBox();
            this.cbInputType = new System.Windows.Forms.ComboBox();
            this.txtGammap = new System.Windows.Forms.TextBox();
            this.txtKi = new System.Windows.Forms.TextBox();
            this.txtKp = new System.Windows.Forms.TextBox();
            this.txtAmpMin = new System.Windows.Forms.TextBox();
            this.txtAmpMax = new System.Windows.Forms.TextBox();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.textBox5 = new System.Windows.Forms.TextBox();
            this.textBox6 = new System.Windows.Forms.TextBox();
            this.textBox7 = new System.Windows.Forms.TextBox();
            this.textBox8 = new System.Windows.Forms.TextBox();
            this.textBox9 = new System.Windows.Forms.TextBox();
            this.serialPort = new System.IO.Ports.SerialPort(this.components);
            this.btnBrake = new System.Windows.Forms.Button();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.txtGammai = new System.Windows.Forms.TextBox();
            this.cbPort = new System.Windows.Forms.ComboBox();
            this.lblStatus = new System.Windows.Forms.Label();
            this.SuspendLayout();
            // 
            // zedGraph1
            // 
            this.zedGraph1.Location = new System.Drawing.Point(-3, 182);
            this.zedGraph1.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.zedGraph1.Name = "zedGraph1";
            this.zedGraph1.ScrollGrace = 0D;
            this.zedGraph1.ScrollMaxX = 0D;
            this.zedGraph1.ScrollMaxY = 0D;
            this.zedGraph1.ScrollMaxY2 = 0D;
            this.zedGraph1.ScrollMinX = 0D;
            this.zedGraph1.ScrollMinY = 0D;
            this.zedGraph1.ScrollMinY2 = 0D;
            this.zedGraph1.Size = new System.Drawing.Size(1032, 262);
            this.zedGraph1.TabIndex = 0;
            // 
            // zedGraph2
            // 
            this.zedGraph2.Location = new System.Drawing.Point(-3, 442);
            this.zedGraph2.Margin = new System.Windows.Forms.Padding(4, 4, 4, 4);
            this.zedGraph2.Name = "zedGraph2";
            this.zedGraph2.ScrollGrace = 0D;
            this.zedGraph2.ScrollMaxX = 0D;
            this.zedGraph2.ScrollMaxY = 0D;
            this.zedGraph2.ScrollMaxY2 = 0D;
            this.zedGraph2.ScrollMinX = 0D;
            this.zedGraph2.ScrollMinY = 0D;
            this.zedGraph2.ScrollMinY2 = 0D;
            this.zedGraph2.Size = new System.Drawing.Size(1032, 264);
            this.zedGraph2.TabIndex = 1;
            // 
            // btnSet
            // 
            this.btnSet.BackColor = System.Drawing.SystemColors.MenuBar;
            this.btnSet.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.btnSet.Location = new System.Drawing.Point(316, 145);
            this.btnSet.Margin = new System.Windows.Forms.Padding(2);
            this.btnSet.Name = "btnSet";
            this.btnSet.Size = new System.Drawing.Size(206, 28);
            this.btnSet.TabIndex = 41;
            this.btnSet.Text = "Set";
            this.btnSet.UseVisualStyleBackColor = false;
            this.btnSet.Click += new System.EventHandler(this.btnSet_Click);
            // 
            // progressBar1
            // 
            this.progressBar1.BackColor = System.Drawing.SystemColors.MenuBar;
            this.progressBar1.Location = new System.Drawing.Point(428, 1);
            this.progressBar1.Margin = new System.Windows.Forms.Padding(4);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(174, 28);
            this.progressBar1.TabIndex = 40;
            // 
            // btn_Connect
            // 
            this.btn_Connect.BackColor = System.Drawing.SystemColors.MenuBar;
            this.btn_Connect.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.btn_Connect.Location = new System.Drawing.Point(177, 1);
            this.btn_Connect.Margin = new System.Windows.Forms.Padding(4);
            this.btn_Connect.Name = "btn_Connect";
            this.btn_Connect.Size = new System.Drawing.Size(100, 28);
            this.btn_Connect.TabIndex = 39;
            this.btn_Connect.Text = "Connect";
            this.btn_Connect.UseVisualStyleBackColor = false;
            this.btn_Connect.Click += new System.EventHandler(this.btn_Connect_Click);
            // 
            // txtSetpoint
            // 
            this.txtSetpoint.Location = new System.Drawing.Point(316, 87);
            this.txtSetpoint.Margin = new System.Windows.Forms.Padding(2);
            this.txtSetpoint.Name = "txtSetpoint";
            this.txtSetpoint.Size = new System.Drawing.Size(109, 22);
            this.txtSetpoint.TabIndex = 38;
            // 
            // textBox22
            // 
            this.textBox22.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox22.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox22.Location = new System.Drawing.Point(316, 47);
            this.textBox22.Margin = new System.Windows.Forms.Padding(2);
            this.textBox22.Name = "textBox22";
            this.textBox22.Size = new System.Drawing.Size(108, 15);
            this.textBox22.TabIndex = 37;
            this.textBox22.Text = "Tốc độ đặt (rpm)";
            // 
            // textBox18
            // 
            this.textBox18.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox18.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox18.Location = new System.Drawing.Point(777, 45);
            this.textBox18.Margin = new System.Windows.Forms.Padding(2);
            this.textBox18.Name = "textBox18";
            this.textBox18.Size = new System.Drawing.Size(182, 15);
            this.textBox18.TabIndex = 36;
            this.textBox18.Text = "Tốc độ mô hình chuẩn (rpm)";
            // 
            // txtplant
            // 
            this.txtplant.Location = new System.Drawing.Point(795, 134);
            this.txtplant.Margin = new System.Windows.Forms.Padding(2);
            this.txtplant.Name = "txtplant";
            this.txtplant.ReadOnly = true;
            this.txtplant.Size = new System.Drawing.Size(149, 22);
            this.txtplant.TabIndex = 35;
            // 
            // txtref
            // 
            this.txtref.Location = new System.Drawing.Point(795, 69);
            this.txtref.Margin = new System.Windows.Forms.Padding(2);
            this.txtref.Name = "txtref";
            this.txtref.ReadOnly = true;
            this.txtref.Size = new System.Drawing.Size(149, 22);
            this.txtref.TabIndex = 34;
            // 
            // textBox13
            // 
            this.textBox13.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox13.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox13.Location = new System.Drawing.Point(777, 106);
            this.textBox13.Margin = new System.Windows.Forms.Padding(2);
            this.textBox13.Name = "textBox13";
            this.textBox13.Size = new System.Drawing.Size(171, 15);
            this.textBox13.TabIndex = 33;
            this.textBox13.Text = "Tốc độ mô hình thực (rpm)";
            // 
            // textBox15
            // 
            this.textBox15.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox15.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox15.Location = new System.Drawing.Point(25, 8);
            this.textBox15.Margin = new System.Windows.Forms.Padding(2);
            this.textBox15.Name = "textBox15";
            this.textBox15.Size = new System.Drawing.Size(117, 15);
            this.textBox15.TabIndex = 32;
            this.textBox15.Text = "Plant Input/Output";
            // 
            // textBox16
            // 
            this.textBox16.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox16.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.textBox16.Location = new System.Drawing.Point(-3, 17);
            this.textBox16.Margin = new System.Windows.Forms.Padding(2);
            this.textBox16.Multiline = true;
            this.textBox16.Name = "textBox16";
            this.textBox16.Size = new System.Drawing.Size(1031, 169);
            this.textBox16.TabIndex = 31;
            // 
            // cbControllerType
            // 
            this.cbControllerType.FormattingEnabled = true;
            this.cbControllerType.Items.AddRange(new object[] {
            "PID",
            "PI-MRAS"});
            this.cbControllerType.Location = new System.Drawing.Point(168, 45);
            this.cbControllerType.Margin = new System.Windows.Forms.Padding(2);
            this.cbControllerType.Name = "cbControllerType";
            this.cbControllerType.Size = new System.Drawing.Size(111, 24);
            this.cbControllerType.TabIndex = 42;
            this.cbControllerType.SelectedIndexChanged += new System.EventHandler(this.cbControllerType_SelectedIndexChanged);
            // 
            // cbInputType
            // 
            this.cbInputType.FormattingEnabled = true;
            this.cbInputType.Items.AddRange(new object[] {
            "Setpoint",
            "Sine",
            "Pulse"});
            this.cbInputType.Location = new System.Drawing.Point(168, 87);
            this.cbInputType.Margin = new System.Windows.Forms.Padding(2);
            this.cbInputType.Name = "cbInputType";
            this.cbInputType.Size = new System.Drawing.Size(111, 24);
            this.cbInputType.TabIndex = 43;
            this.cbInputType.SelectedIndexChanged += new System.EventHandler(this.cbInputType_SelectedIndexChanged);
            // 
            // txtGammap
            // 
            this.txtGammap.Location = new System.Drawing.Point(667, 45);
            this.txtGammap.Margin = new System.Windows.Forms.Padding(2);
            this.txtGammap.Name = "txtGammap";
            this.txtGammap.Size = new System.Drawing.Size(85, 22);
            this.txtGammap.TabIndex = 44;
            // 
            // txtKi
            // 
            this.txtKi.Location = new System.Drawing.Point(495, 72);
            this.txtKi.Margin = new System.Windows.Forms.Padding(2);
            this.txtKi.Name = "txtKi";
            this.txtKi.Size = new System.Drawing.Size(81, 22);
            this.txtKi.TabIndex = 46;
            // 
            // txtKp
            // 
            this.txtKp.Location = new System.Drawing.Point(495, 44);
            this.txtKp.Margin = new System.Windows.Forms.Padding(2);
            this.txtKp.Name = "txtKp";
            this.txtKp.Size = new System.Drawing.Size(81, 22);
            this.txtKp.TabIndex = 47;
            // 
            // txtAmpMin
            // 
            this.txtAmpMin.Location = new System.Drawing.Point(168, 123);
            this.txtAmpMin.Margin = new System.Windows.Forms.Padding(2);
            this.txtAmpMin.Name = "txtAmpMin";
            this.txtAmpMin.Size = new System.Drawing.Size(111, 22);
            this.txtAmpMin.TabIndex = 48;
            // 
            // txtAmpMax
            // 
            this.txtAmpMax.Location = new System.Drawing.Point(168, 157);
            this.txtAmpMax.Margin = new System.Windows.Forms.Padding(2);
            this.txtAmpMax.Name = "txtAmpMax";
            this.txtAmpMax.Size = new System.Drawing.Size(111, 22);
            this.txtAmpMax.TabIndex = 49;
            // 
            // textBox1
            // 
            this.textBox1.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox1.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox1.Location = new System.Drawing.Point(457, 45);
            this.textBox1.Margin = new System.Windows.Forms.Padding(2);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(34, 15);
            this.textBox1.TabIndex = 51;
            this.textBox1.Text = "Kp";
            // 
            // textBox3
            // 
            this.textBox3.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox3.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox3.Location = new System.Drawing.Point(457, 75);
            this.textBox3.Margin = new System.Windows.Forms.Padding(2);
            this.textBox3.Name = "textBox3";
            this.textBox3.Size = new System.Drawing.Size(34, 15);
            this.textBox3.TabIndex = 52;
            this.textBox3.Text = "Ki";
            // 
            // textBox5
            // 
            this.textBox5.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox5.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox5.Location = new System.Drawing.Point(592, 47);
            this.textBox5.Margin = new System.Windows.Forms.Padding(2);
            this.textBox5.Name = "textBox5";
            this.textBox5.Size = new System.Drawing.Size(71, 15);
            this.textBox5.TabIndex = 54;
            this.textBox5.Text = "Gamma_p";
            // 
            // textBox6
            // 
            this.textBox6.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox6.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox6.Location = new System.Drawing.Point(25, 47);
            this.textBox6.Margin = new System.Windows.Forms.Padding(2);
            this.textBox6.Name = "textBox6";
            this.textBox6.Size = new System.Drawing.Size(139, 15);
            this.textBox6.TabIndex = 55;
            this.textBox6.Text = "Chọn kiểu điều khiển";
            // 
            // textBox7
            // 
            this.textBox7.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox7.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox7.Location = new System.Drawing.Point(25, 87);
            this.textBox7.Margin = new System.Windows.Forms.Padding(2);
            this.textBox7.Name = "textBox7";
            this.textBox7.Size = new System.Drawing.Size(139, 15);
            this.textBox7.TabIndex = 56;
            this.textBox7.Text = "Chọn kiểu ngõ vào";
            // 
            // textBox8
            // 
            this.textBox8.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox8.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox8.Location = new System.Drawing.Point(25, 125);
            this.textBox8.Margin = new System.Windows.Forms.Padding(2);
            this.textBox8.Name = "textBox8";
            this.textBox8.Size = new System.Drawing.Size(118, 15);
            this.textBox8.TabIndex = 57;
            this.textBox8.Text = "Tốc độ min (rpm)";
            // 
            // textBox9
            // 
            this.textBox9.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox9.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox9.Location = new System.Drawing.Point(25, 157);
            this.textBox9.Margin = new System.Windows.Forms.Padding(2);
            this.textBox9.Name = "textBox9";
            this.textBox9.Size = new System.Drawing.Size(118, 15);
            this.textBox9.TabIndex = 58;
            this.textBox9.Text = "Tốc độ max (rpm)";
            // 
            // serialPort
            // 
            this.serialPort.BaudRate = 14400;
            this.serialPort.PortName = "COM13";
            this.serialPort.ReadBufferSize = 2048;
            this.serialPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.Serial_DataReceived);
            // 
            // btnBrake
            // 
            this.btnBrake.BackColor = System.Drawing.Color.Lime;
            this.btnBrake.FlatStyle = System.Windows.Forms.FlatStyle.Popup;
            this.btnBrake.Location = new System.Drawing.Point(559, 145);
            this.btnBrake.Margin = new System.Windows.Forms.Padding(2);
            this.btnBrake.Name = "btnBrake";
            this.btnBrake.Size = new System.Drawing.Size(206, 28);
            this.btnBrake.TabIndex = 59;
            this.btnBrake.Text = "ON";
            this.btnBrake.UseVisualStyleBackColor = false;
            this.btnBrake.Click += new System.EventHandler(this.btnBrake_Click);
            // 
            // textBox2
            // 
            this.textBox2.BackColor = System.Drawing.SystemColors.MenuBar;
            this.textBox2.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBox2.Location = new System.Drawing.Point(592, 77);
            this.textBox2.Margin = new System.Windows.Forms.Padding(2);
            this.textBox2.Name = "textBox2";
            this.textBox2.Size = new System.Drawing.Size(71, 15);
            this.textBox2.TabIndex = 60;
            this.textBox2.Text = "Gamma_i";
            // 
            // txtGammai
            // 
            this.txtGammai.Location = new System.Drawing.Point(667, 77);
            this.txtGammai.Margin = new System.Windows.Forms.Padding(2);
            this.txtGammai.Name = "txtGammai";
            this.txtGammai.Size = new System.Drawing.Size(85, 22);
            this.txtGammai.TabIndex = 61;
            // 
            // cbPort
            // 
            this.cbPort.FormattingEnabled = true;
            this.cbPort.Location = new System.Drawing.Point(290, 3);
            this.cbPort.Name = "cbPort";
            this.cbPort.Size = new System.Drawing.Size(121, 24);
            this.cbPort.TabIndex = 62;
            // 
            // lblStatus
            // 
            this.lblStatus.BackColor = System.Drawing.Color.Transparent;
            this.lblStatus.ForeColor = System.Drawing.Color.Transparent;
            this.lblStatus.Location = new System.Drawing.Point(609, 3);
            this.lblStatus.Name = "lblStatus";
            this.lblStatus.Size = new System.Drawing.Size(174, 28);
            this.lblStatus.TabIndex = 63;
            this.lblStatus.Text = "label1";
            this.lblStatus.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(1033, 675);
            this.Controls.Add(this.lblStatus);
            this.Controls.Add(this.cbPort);
            this.Controls.Add(this.txtGammai);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.btnBrake);
            this.Controls.Add(this.textBox9);
            this.Controls.Add(this.textBox8);
            this.Controls.Add(this.textBox7);
            this.Controls.Add(this.textBox6);
            this.Controls.Add(this.textBox5);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.txtAmpMax);
            this.Controls.Add(this.txtAmpMin);
            this.Controls.Add(this.txtKp);
            this.Controls.Add(this.txtKi);
            this.Controls.Add(this.txtGammap);
            this.Controls.Add(this.cbInputType);
            this.Controls.Add(this.cbControllerType);
            this.Controls.Add(this.btnSet);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.btn_Connect);
            this.Controls.Add(this.txtSetpoint);
            this.Controls.Add(this.textBox22);
            this.Controls.Add(this.textBox18);
            this.Controls.Add(this.txtplant);
            this.Controls.Add(this.txtref);
            this.Controls.Add(this.textBox13);
            this.Controls.Add(this.textBox15);
            this.Controls.Add(this.zedGraph1);
            this.Controls.Add(this.zedGraph2);
            this.Controls.Add(this.textBox16);
            this.Margin = new System.Windows.Forms.Padding(2);
            this.Name = "Form1";
            this.Text = "PI & PI - MRAS UI CONTROL";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private ZedGraph.ZedGraphControl zedGraph1;
        private ZedGraph.ZedGraphControl zedGraph2;
        private System.Windows.Forms.Button btnSet;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Button btn_Connect;
        private System.Windows.Forms.TextBox txtSetpoint;
        private System.Windows.Forms.TextBox textBox22;
        private System.Windows.Forms.TextBox textBox18;
        private System.Windows.Forms.TextBox txtplant;
        private System.Windows.Forms.TextBox txtref;
        private System.Windows.Forms.TextBox textBox13;
        private System.Windows.Forms.TextBox textBox15;
        private System.Windows.Forms.TextBox textBox16;
        private System.Windows.Forms.ComboBox cbControllerType;
        private System.Windows.Forms.ComboBox cbInputType;
        private System.Windows.Forms.TextBox txtGammap;
        private System.Windows.Forms.TextBox txtKi;
        private System.Windows.Forms.TextBox txtKp;
        private System.Windows.Forms.TextBox txtAmpMin;
        private System.Windows.Forms.TextBox txtAmpMax;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.TextBox textBox5;
        private System.Windows.Forms.TextBox textBox6;
        private System.Windows.Forms.TextBox textBox7;
        private System.Windows.Forms.TextBox textBox8;
        private System.Windows.Forms.TextBox textBox9;
        private System.IO.Ports.SerialPort serialPort;
        private System.Windows.Forms.Button btnBrake;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox txtGammai;
        private System.Windows.Forms.ComboBox cbPort;
        private System.Windows.Forms.Label lblStatus;
    }
}

