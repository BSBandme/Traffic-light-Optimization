package my_package;

import java.util.ArrayList;
import java.util.List;

/***
 * 
 * ��ͨʮ��·��
 *
 */
public class TrafficCrossroad 
{
	public String id; //·��id
	public String[] neighbours; //���ڵ�·��,˳��Ϊ��������
	
	public int[] currentFlow; //��ǰ����
	public int currentTime; //��ǰʱ��
	public List<int[] > flowHistory; //��ʷ����
	public int[] lightSettingHistory; //��ʷ�趨״̬
	public int[][] flowAdd; //ÿ��ʱ���ͻȻ���ֵ�������Ԥ����
	
	public void setNeightbours(String left, String up, String right, String down)
	{
		this.neighbours = new String[4];
		neighbours[0] = left;
		neighbours[1] = up;
		neighbours[2] = right;
		neighbours[3] = down;
	}
	
	public TrafficCrossroad(String id)
	{
		this.id = id;
		
		this.flowAdd = new int[Constants.MAX_TIME+Constants.ESTIMATE_INTERVAL][];
		for(int i=0;i<flowAdd.length;i++)
		{
			flowAdd[i] = new int[4];
		}
		
		this.lightSettingHistory = new int[Constants.MAX_TIME+Constants.ESTIMATE_INTERVAL];
		flowHistory = new ArrayList<int[]>();
	}
	
	public void setLight(int setting, int time)
	{
		this.lightSettingHistory[time] = setting;
	}
}
