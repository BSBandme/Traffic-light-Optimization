package my_package;

import java.io.File;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.FileOutputStream;  
import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.ArrayList;

public class Main 
{
	/***
	 * 将输入的字符串转换为流量map
	 * 
	 * @param traffic - 交通结构图
	 * @param line - 输出字符串
	 * @return 流量map
	 */
//	public static List<Integer> sum;
	
	public static Map<String,int[]> String2Flow(TrafficGraph traffic, String line)
	{
//		int rsum = 0;
		traffic.sumflow = 0;
		Map<String,int[]> ret = new HashMap<String,int[]>();
		
		for(String id : traffic.crosses.keySet())
		{
			ret.put(id, new int[4]);
		}
		
		String[] parts = line.split(";");
		
		for(String part : parts)
		{
			String[] pp = part.split(",");
			String id = pp[0];
			String frmId = pp[1];
			int flow = Integer.parseInt(pp[2]);
			traffic.sumflow += flow;
//			rsum += flow; 
			
			TrafficCrossroad cr = traffic.crosses.get(id);
			
			for(int i=0;i<cr.neighbours.length;i++)
			{
				if (cr.neighbours[i].compareTo(frmId)==0)
				{
					ret.get(id)[i] = flow;
				}
			}
		}
//		sum.add(rsum);
//		System.out.println(sum);
		
		return ret;
	}
	
	/***
	 * 将路口的红绿灯状态转换为字符串
	 * 
	 * @param traffic - 交通结构图
	 * @param time - 当前时间
	 * @return 字符串
	 */
	public static String OutputLightSetting(TrafficGraph traffic, int time)
	{
		StringBuffer sb = new StringBuffer();
		int cnt = 0;
		for(Map.Entry<String, TrafficCrossroad> entry : traffic.crosses.entrySet())
		{
			String cid = entry.getKey();
			TrafficCrossroad cross = entry.getValue();
			
			int setting = cross.lightSettingHistory[time];
			
			int[] status = new int[12];
			if(setting < 1 << 12) {
				if ( setting % 2 == 0)
				{//水平方向
					status[0] = 1;
					status[1] = 1;
					status[2] = 1;
					status[3] = 0;
					status[4] = 1;
					status[5] = 0;
					status[6] = 1;
					status[7] = 1;
					status[8] = 1;
					status[9] = 0;
					status[10] = 1;
					status[11] = 0;
				}
				else
				{//垂直方向
					status[0] = 0;
					status[1] = 1;
					status[2] = 0;
					status[3] = 1;
					status[4] = 1;
					status[5] = 1;
					status[6] = 0;
					status[7] = 1;
					status[8] = 0;
					status[9] = 1;
					status[10] = 1;
					status[11] = 1;				
				}
				if(setting > 1) {
					for(int i = 2; i < 12; i += 3)
						status[i] = 1;
				}
			} else{
				for(int i = 0; i < 12; i++) 
					if((setting & (1 << i)) != 0)
						status[i] = 1;
			}
				
			
			for(int i=0;i<4;i++)
			{
				String dstId = cross.neighbours[i];
				if(dstId.compareTo(Constants.LIGHT_NONE)!=0)
				{
					if ( cnt> 0)
					{
						sb.append(";");
					}
					cnt++;
					sb.append(cid + "," + dstId);
					for(int j=0;j<3;j++)
					{
						sb.append("," + status[i*3+j]);
						while(status[i*3+j] > 1) {
							System.out.println(1);
						}
					}
				}
			}
		}
		
		return sb.toString();
	}
	
	public static String Process(String line, int time, TrafficGraph traffic)
	{
		Map<String,int[]> flow = String2Flow(traffic,line);
		Algorithms.Solve(traffic, flow, time);
		
		return OutputLightSetting(traffic,time);	
	}
	
	public static String change(String k) {
		if(k == null) return k;
		String[] arr = k.split("\t");
		return arr[arr.length - 1].trim();
	}

	
	public static void main(String args[]) throws IOException
	{
		
		//initialize
//		sum = new ArrayList<Integer>();
		TrafficGraph traffic = new TrafficGraph();
		BufferedReader reader = new BufferedReader(new InputStreamReader(
				Main.class.getResourceAsStream(Constants.FILENAME_TRAFFIC)));
		BufferedReader readerFlow = new BufferedReader( new InputStreamReader(
				Main.class.getResourceAsStream(Constants.FILENAME_FLOW_ADD)));

		//读入红绿灯的结构图
		traffic.load(reader);
		//读入每个时刻突然出现的流量
		traffic.loadFlowAdd(readerFlow);
		reader.close();
		readerFlow.close();
		
		//main process
		BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
		
//		//debug------------------------------------------------------------------------------
//		File f = new File(Constants.FILENAME_Debug);
//		FileOutputStream fop = new FileOutputStream(f);
//		// 构建FileOutputStream对象,文件不存在会自动新建
//		OutputStreamWriter bw_debug = new OutputStreamWriter(fop, "UTF-8");
//		BufferedReader br = new BufferedReader( new InputStreamReader(
//				Main.class.getResourceAsStream("input.txt")));
//		//debug------------------------------------------------------------------------------
//		
		
		
		
//t1,t2,18;t2,t1,20;t2,t3,21;t3,t2,16;t3,t4,21;t4,t3,19;t1,t4,23;t4,t1,24;t5,t1,25;t1,t5,35;t5,t2,18;t2,t5,16;t5,t3,15;t3,t5,23;t5,t4,20;t4,t5,21
//t1,t2,0;t2,t1,0;t2,t3,0;t3,t2,0;t3,t4,0;t4,t3,0;t1,t4,0;t4,t1,0;t5,t1,0;t1,t5,0;t5,t2,0;t2,t5,0;t5,t3,0;t3,t5,0;t5,t4,0;t4,t5,0		
		
		String flows_str = br.readLine();
		flows_str = change(flows_str);
		int time = 0;
		traffic.pen = 0;
		while(!"end".equalsIgnoreCase(flows_str))
		{
			//TODO  你的代码,注意，数据输出需保证一行输出，除了数据结果，请不要将任何无关数据或异常打印输出
			System.out.println(Process(flows_str,time,traffic));
	        //获取下一个时间段的流量
			flows_str = br.readLine();
			
//			//debug-------------------------------------------------------------------------
//			if(flows_str == null) {
//				break;
//			}
//			if(!"end".equalsIgnoreCase(flows_str))
//				flows_str = change(flows_str);
//			bw_debug.append(String.valueOf((int)traffic.pen));
//			bw_debug.append(" " + String.valueOf(traffic.sumflow) + "\r\n");
//			//debug-------------------------------------------------------------------------
			
			time++;

//			if(sum.size() > 1 && sum.get(sum.size() - 1) < sum.get(sum.size() - 2)) {
//				System.out.println(1235);
//			}
		}
		
//		//debug----------------------------------------------------------------------------
//		bw_debug.close();
//		//debug----------------------------------------------------------------------------
	}
}
