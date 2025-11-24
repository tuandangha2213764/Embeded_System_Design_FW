function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["current_speed"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["current_speed_PImras"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["current_speed_model"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["error"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["error_PImras"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["gi"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["gp"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["ki"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["kp"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["rtDW"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 84};
	 this.metricsArray.var["set_point"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["set_point_mras"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["set_point_sine"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["signal_control"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.var["signal_controlPImras"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	size: 8};
	 this.metricsArray.fcn["Motor_control_initialize"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["Motor_control_step"] = {file: "C:\\Users\\DELL\\Desktop\\Keil_C_motorcontrol\\New_control\\MDK-ARM\\Motor_control_ert_rtw\\Motor_control.c",
	stack: 16,
	stackTotal: 16};
	 this.metricsArray.fcn["sin"] = {file: "C:\\Program Files\\MATLAB\\R2017b\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data; }; 
	 this.codeMetricsSummary = '<a href="Motor_control_metrics.html">Global Memory: 196(bytes) Maximum Stack: 16(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();
