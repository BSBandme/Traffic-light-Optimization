package my_package;


import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/***
 * 
 * ��ͨ�ṹͼ
 *
 */
public class TrafficGraph 
{
	public Map<String, TrafficCrossroad> crosses; //���нڵ�
	double pen;
	int sumflow;
	
	TrafficGraph()
	{
		this.crosses = new HashMap<String,TrafficCrossroad>();
	}
	
	
	/***
	 * ��reader�ж��뽻ͨ�ṹ
	 * 
	 * @param reader - ����
	 * @throws IOException
	 */
	public void load(BufferedReader reader) throws IOException
	{
		Map<String, List<String[]> > preMap = new HashMap<String, List<String[]> >();

		String line  = "";
		
		while(line != null)
		{
			line = reader.readLine();
			if (line == null)
			{
				break;
			}
			line = line.trim();
			String[] parts = line.split(",");
			String[] otherParts = Arrays.copyOfRange(parts,1,parts.length);
			
			if (parts.length != 5)
			{
				System.out.println(line);
				reader.close();
				throw new RuntimeException("logic error" + "part's length:" + parts.length 
						+ line);
			}
			if (preMap.containsKey(parts[0]))
			{
				preMap.get(parts[0]).add(otherParts);
			}
			else
			{
				List<String[]> lists = new ArrayList<String[]>();
				lists.add(otherParts);
				preMap.put(parts[0], lists);
			}
		}
		
		reader.close();
		
		for(Map.Entry<String, List<String[]> > entry : preMap.entrySet())
		{
			String cid = entry.getKey().trim();
			List<String[]> lists = entry.getValue();
			String left = lists.get(0)[0].trim();
			String up = Constants.LIGHT_NONE;
			String right = Constants.LIGHT_NONE;
			String down = Constants.LIGHT_NONE;
			
			for(int i=1;i<lists.size();i++)
			{
				String[] rec = lists.get(i);
				String from = rec[0].trim();
				String leftTarget = rec[1].trim();
				String rightTarget = rec[2].trim();
				String straightTarget = rec[3].trim();
				
				if ( leftTarget.compareTo(left)==0)
				{
					down = from;
				}
				if ( rightTarget.compareTo(left)==0)
				{
					up = from;
				}
				if ( straightTarget.compareTo(left)==0)
				{
					right = from;
				}
			}
			
			TrafficCrossroad cross = new TrafficCrossroad(cid);
			cross.setNeightbours(left, up, right, down);
			crosses.put(cid, cross);
		}   		
		
	}
	public void load(String filename) throws IOException
	{
		BufferedReader reader = new BufferedReader(new FileReader(filename));
		this.load(reader);		
	}
	
	
	/***
	 * �������·��ͻȻ���ֵ�����
	 * 
	 * @param reader - ����
	 * @throws IOException
	 */
	public void loadFlowAdd(BufferedReader reader) throws IOException
	{	
		String line = reader.readLine();
		
		while(line != null)
		{
			line = line.trim();
			String parts[] = line.split(",");
			String frmId = parts[1];
			String dstId = parts[0];
			TrafficCrossroad vertex = this.crosses.get(dstId);
			if (vertex != null)
			{
				String[] flows = Arrays.copyOfRange(parts,2,parts.length);
				for(int i=0;i<4;i++)
				{
					if(vertex.neighbours[i].compareTo(frmId)==0)
					{
						for(int j=0;j<flows.length;j++)
						{
							vertex.flowAdd[j][i] = Integer.parseInt(flows[j]);
						}
					}
				}
			}
			line = reader.readLine();
		}		
	}
	public void loadFlowAdd(String filename) throws IOException
	{	
		BufferedReader reader = new BufferedReader(new FileReader(filename));
		this.loadFlowAdd(reader);
		reader.close();
	}
	
	
	/***
	 * ȡ�õ�ǰ����
	 * 
	 * @return
	 */
	public Map<String,float[]> getCurrentFlow()
	{
		Map<String,float[]> ret = new HashMap<String,float[]>();
		for(TrafficCrossroad cross: this.crosses.values())
		{
			float[] f = new float[4];
			for(int i=0;i<f.length;i++)
			{
				f[i] = (float) cross.currentFlow[i];
			}
			ret.put(cross.id, f);
		}
		return ret;
	}
	
	/***
	 * ȡ�õ�timeʱ�̵�����
	 * 
	 * @param time
	 * @return
	 */
	public Map<String,int[]> getFlow(int time)
	{
		Map<String,int[]> ret = new HashMap<String,int[]>();
		for(TrafficCrossroad cross: this.crosses.values())
		{
			int[] f = new int[4];
			for(int i=0;i<f.length;i++)
			{
				f[i] = (int) cross.flowHistory.get(time)[i];
			}
			ret.put(cross.id, f);
		}
		return ret;		
	}
	
	/***
	 * ȡ�ýڵ�cid�ĵ�timeʱ�̵�����
	 * 
	 * @param cid
	 * @param time
	 * @return
	 */
	public int[] getFlowAdd(String cid, int time)
	{
		return this.crosses.get(cid).flowAdd[time];
	}
	
	/***
	 * ����timeʱ�̵���������flow
	 * 
	 * @param flow
	 * @param time
	 */
	public void flowAdd(Map<String,float[]>flow, int time)
	{
		for(Map.Entry<String, float[]>entry: flow.entrySet())
		{
			String cid = entry.getKey();
			float[] cflow = entry.getValue();
			int[] flowAdd = this.getFlowAdd(cid, time);
			
			Utils.ArrayAdd(cflow, flowAdd);
		}
	}
	public void flowAdd(Map<String,float[]>flow, int time, float scale)
	{
		for(Map.Entry<String, float[]>entry: flow.entrySet())
		{
			String cid = entry.getKey();
			float[] cflow = entry.getValue();
			int[] flowAdd = this.getFlowAdd(cid, time);
			
//			Utils.ArrayAdd(Utils.ArrayScale(cflow, 1 - scale), flowAdd, scale);
			Utils.ArrayAdd(cflow, flowAdd, scale);
		}
	}
	
	public void setLight(String cid, int setting, int time)
	{
		this.crosses.get(cid).setLight(setting, time);
	}
	
	public void setLight(Map<String,Integer> setting, int time)
	{
		for(Map.Entry<String, Integer> entry : setting.entrySet())
		{
			this.setLight(entry.getKey(), entry.getValue(),time);
		}
	}
	
	public void saveCurrentFlow()
	{
		for(Map.Entry<String, TrafficCrossroad> entry: crosses.entrySet())
		{
			TrafficCrossroad cross = entry.getValue();
			cross.flowHistory.add(cross.currentFlow.clone());
		}
	}
	
	public void setCurrentFlow(Map<String,int[]> flow, int time)
	{
		for(Map.Entry<String, TrafficCrossroad> entry: this.crosses.entrySet())
		{
			String cid = entry.getKey();
			TrafficCrossroad cross = entry.getValue();
			cross.currentFlow = flow.get(cid).clone();
//			if(time == 0) 
//				cross.currentFlow = flow.get(cid).clone();
//			else 
//				Utils.ArrayAdd(cross.currentFlow, flow.get(cid).clone());
		}
	}
	
	/***
	 * �ҳ�frmId��dstId���ĸ�����
	 * 
	 * @param dstId
	 * @param frmId
	 * @return
	 */
	public int findNeighbourIndex(String dstId, String frmId)
	{
		TrafficCrossroad cr = this.crosses.get(dstId);
		for(int i=0;i<4;i++)
		{
			if ( cr.neighbours[i].compareTo(frmId)==0)
			{
				return i;
			}
		}
		
		return -1;
	}
	
	public float[] getFlow(String frmid, String tarid, float flow, float[] rprob) {
		float[] prob = rprob.clone(), through = Constants.MAX_THROUGH.clone();
		float[] ans = new float[3];
		for(int i = 0; i < 3; i++) ans[i] = 0;
		if(frmid == Constants.LIGHT_NONE) return ans;
		TrafficCrossroad temp = this.crosses.get(tarid);
		for(int i = 0; i < 4; i++) if(frmid == temp.neighbours[i]) {
			int no = -1;
			for(int j = 1; j < 4; j++) {
				if(temp.neighbours[(i + j) % 4] == Constants.LIGHT_NONE) {
					no = j - 1;
					break;
				}
			}
			if(no == 0) {
				prob[2] += prob[0];
				prob[0] = 0;
//				through[2] += through[0];
//				through[0] = 0;
			}
			if(no == 1) {
				prob[2] += prob[1] / 2;
				prob[0] += prob[1] / 2;
				prob[1] = 0;
//				through[2] += through[1] / 2;
//				through[0] += through[0] / 2;
//				through[1] = 0;
			}
			if(no == 2) {
				prob[0] += prob[2];
				prob[2] = 0;
//				through[0] += through[2];
//				through[2] = 0;
			}
			for(int j = 0; j < 3; j++) {
				ans[j] = Math.min(through[j], prob[j] * flow);
			}
			break;
		}
		return ans;
	}
	/***
	 * ���ݵ�ǰʱ��time������һ�ε�����
	 * 
	 * @param time
	 * @return
	 */
	public Map<String,int[]> computeNextFlow(int time)
	{
		Map<String,int[]> flow = this.getFlow(time);
		
		for(TrafficCrossroad cross : this.crosses.values())
		{
			String cid = cross.id;
			int setting = cross.lightSettingHistory[time];
			CrossFlow cf = Algorithms.CalcCrossFlow(cross.flowHistory.get(time),
					Constants.TURN_PROBA);
			if ( setting==0)
			{
				cf.flowD2L = 0;
				cf.flowD2U = 0;
				cf.flowU2D = 0;
				cf.flowU2R = 0;
			}
			else if (setting==1)
			{
				cf.flowL2U = 0;
				cf.flowL2R = 0;
				cf.flowR2L = 0;
				cf.flowR2D = 0;
			}
			
			int[] f = flow.get(cid);
			f[0] -= cf.flowL2D+cf.flowL2R+cf.flowL2U;
			f[1] -= cf.flowU2D+cf.flowU2L+cf.flowU2R;
			f[2] -= cf.flowR2D+cf.flowR2L+cf.flowR2U;
			f[3] -= cf.flowD2L+cf.flowD2R+cf.flowD2U;
			
			int[] nis = new int[4];
			for(int i=0;i<4;i++)
			{
				nis[i] = findNeighbourIndex(cross.neighbours[0], cid);
			}
			
			if ( cross.neighbours[0].compareTo(Constants.LIGHT_NONE)!=0)
			{
				flow.get(cross.neighbours[0])[nis[0]] += cf.flowD2L+cf.flowD2R+cf.flowD2U;
			}
			
			
		}
		
		return null;
	}
}
