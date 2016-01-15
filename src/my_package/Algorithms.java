package my_package;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class Algorithms
{
	static Random rand = new Random();
	static float eps = (float)1e-5;
	
	/***
	 * 将现有流量复制一份
	 * 
	 * @param flow
	 * @return a copy
	 */
	public static Map<String,float[]> CopyFlow(Map<String,float[]> flow)
	{
		Map<String,float[]> ret = new HashMap<String,float[]>();
		
		for(Map.Entry<String, float[]> entry: flow.entrySet())
		{
			ret.put(entry.getKey(), entry.getValue().clone());
		}
		
		return ret;
	}
	
	/***
	 * 根据概率计算流量
	 * 
	 * @param x - 流入量
	 * @param proba - 概率alpha,beta,gamma
	 * @return
	 */
	public static int[] sampleData(int x, float[] proba)
	{
		int[] ret = new int[proba.length];
		
		/*
		for(int i = 0;i<x;i++)
		{
			double r = rand.nextDouble();
			double p = 0.0;
			for(int j=0;j<proba.length;j++)
			{
				p+=proba[j];
				if ( r<=p)
				{
					ret[j]++;
					break;
				}
			}
		}
		*/
		for(int i=0;i<proba.length;i++)
		{
			ret[i] = (int) Math.ceil(((float)x) * proba[i]);
		}
		
		return ret;
	}
	
	/***
	 * 根据当前四个方向的流量，和转弯概率计算出车辆流动情况
	 * 
	 * @param trafficStatus - 四个方向的流量
	 * @param probaTurn - 转弯概率alpha,beta,gamma
	 * @return 车辆流动情况
	 */
	public static CrossFlow CalcCrossFlow(int[] trafficStatus, float[] probaTurn)
	{
		CrossFlow flow = new CrossFlow();
		int[] t = sampleData(trafficStatus[0],probaTurn);
		
		flow.flowL2U = Math.min((int) Constants.MAX_THROUGH[0],t[0]);
		flow.flowL2R = Math.min((int) Constants.MAX_THROUGH[1],t[1]);
		flow.flowL2D = Math.min((int) Constants.MAX_THROUGH[2],t[2]);
		
		t = sampleData(trafficStatus[1],probaTurn);
		flow.flowU2R = Math.min((int) Constants.MAX_THROUGH[0],t[0]);
		flow.flowU2D = Math.min((int) Constants.MAX_THROUGH[1],t[1]);
		flow.flowU2L = Math.min((int) Constants.MAX_THROUGH[2],t[2]);
		
		t = sampleData(trafficStatus[2],probaTurn);
		flow.flowR2D = Math.min((int) Constants.MAX_THROUGH[0],t[0]);
		flow.flowR2L = Math.min((int) Constants.MAX_THROUGH[1],t[1]);
		flow.flowR2U = Math.min((int) Constants.MAX_THROUGH[2],t[2]);		
		
		t = sampleData(trafficStatus[3],probaTurn);
		flow.flowD2L = Math.min((int) Constants.MAX_THROUGH[0],t[0]);
		flow.flowD2U = Math.min((int) Constants.MAX_THROUGH[1],t[1]);
		flow.flowD2R = Math.min((int) Constants.MAX_THROUGH[2],t[2]);
		
		return flow;
	}
	
	/***
	 * 计算往前走的cost
	 * 
	 * @param ecost - 根据前馈反馈算法计算出的边的权重
	 * @param ef - 车辆流动情况
	 * @return - 第一种状态和第二种状态的cost
	 */
	public static float[] JudgeForwardCost(float[] ecost, CrossFlow ef)
	{
		float[][] forwardCost = new float[2][];
		forwardCost[0] = new float[4];
		forwardCost[1] = new float[4];
		forwardCost[0][0] = ef.flowR2L + ef.flowU2L;
		forwardCost[0][1] = ef.flowL2U + ef.flowR2U;
		forwardCost[0][2] = ef.flowL2R + ef.flowD2R;
		forwardCost[0][3] = ef.flowL2D + ef.flowR2D;
		
		forwardCost[1][0] = ef.flowU2L + ef.flowD2L;
		forwardCost[1][1] = ef.flowD2U + ef.flowR2U;
		forwardCost[1][2] = ef.flowU2R + ef.flowD2R;
		forwardCost[1][3] = ef.flowU2D + ef.flowL2D;
		
		float[] ret = new float[2];
		
		for(int i=0;i<4;i++)
		{
			ret[0] += forwardCost[0][i] * ecost[i];
			ret[1] += forwardCost[1][i] * ecost[i];
		}
		
		return ret;
	}
	
	/***
	 * 计算第一种状态和第二种状态的cost
	 * 
	 * @param flow - 当前4个方向的流量
	 * @param ecost - 根据前馈反馈算法计算出的边的权重
	 * @param ef - 车辆流动情况
	 * @return 第一种状态和第二种状态的cost
	 */
	public static float[] JudgeCost(int[] flow, float[] ecost, CrossFlow ef)
	{
		//int tcost = Utils.ArraySum(flow);
		float[] costs = new float[2];
		costs[0] = ef.flowL2R+ef.flowL2D+ef.flowL2U
				+ef.flowR2L+ef.flowR2U + ef.flowR2D
				+ef.flowU2L+ef.flowD2R;
		costs[1] = ef.flowD2U+ef.flowD2L+ef.flowD2R
				+ef.flowU2D+ef.flowU2L+ef.flowU2R
				+ef.flowL2D+ef.flowR2U;
		
		//costs[0] = Math.max(0,tcost-costs[0]);
		//costs[1] = Math.max(0,tcost-costs[1]);
		costs[0] = -costs[0];
		costs[1] = -costs[1];
		
		float[] fcost = JudgeForwardCost(ecost,ef);
		
		costs[0] += fcost[0] * Constants.LAMBDA_2;
		costs[1] += fcost[1] * Constants.LAMBDA_2;
		
		return costs;
	}
	
	/***
	 * flow中frmId到dstId的流量加上addf
	 * 
	 * @param flow - 当前流量
	 * @param frmId - 来源节点
	 * @param dstId - 目的节点
	 * @param traffic - 交通结构图
	 * @param addf - 流量增加量
	 */
	private static void AddMapFlow(Map<String,float[]> flow, String frmId, String dstId, 
			TrafficGraph traffic, float addf)
	{
		TrafficCrossroad cross = traffic.crosses.get(dstId);
		
		if (cross != null)
		{
			String[] nns = cross.neighbours;
			
			for(int i=0;i<nns.length;i++)
			{
				if (nns[i].compareTo(frmId)==0)
				{
					flow.get(dstId)[i] += addf;
				}
			}
		}
	}
	
	/***
	 * 根据概率计算三个方向的流量
	 * 
	 * @param flow - 流入量
	 * @param turnProba - 转弯概率
	 * @return 三个方向的流量
	 */
	public static float[] CalcTurnFlow(float flow, float[] turnProba)
	{
		float[] turn = Utils.ArrayScale(turnProba, flow);
		for(int i=0;i<turn.length;i++)
		{
			turn[i] = Math.min(Constants.MAX_THROUGH[i], turn[i]);
		}
		return turn;
	}
	
	/***
	 * 根据当前流量计算各个节点的流出量
	 * 
	 * @param traffic - 交通结构图
	 * @param cflow - 当前流量
	 * @param turnProba - 转弯概率
	 * @return 各个节点的流出量
	 */
	public static Map<String,float[]> CalcOutFlow(TrafficGraph traffic, 
			Map<String,float[]> cflow, float[] turnProba)
	{
		Map<String,float[]> ret = new HashMap<String,float[]>();
		for(String cid : traffic.crosses.keySet())
		{
			ret.put(cid, new float[4]);
		}
		
		for(Map.Entry<String, float[]> entry : cflow.entrySet())
		{
			String cid = entry.getKey();
			float[] f = entry.getValue();
			String[] nns = traffic.crosses.get(cid).neighbours;
			
			for(int i=0;i<f.length;i++)
			{
				String frm = cid;
				float[] tf = CalcTurnFlow(f[i],turnProba);
				
				for(int j=0;j<tf.length;j++)
				{
					int idst = (i+j) % 4;
					String dst = nns[idst];
					
					AddMapFlow(ret,frm,dst,traffic,tf[j]);
				}
				f[i] -= Utils.ArraySum(tf);
			}
		}
		
		return ret;
	}
	
	/***
	 * 让流量自由流动，计算下个时间段的各个节点的流量
	 * 
	 * @param traffic 交通图
	 * @param currentFlow 当前流量
	 * @return 下个时间段的流量
	 */
	public static Map<String,float[]> Forward(TrafficGraph traffic, 
			Map<String,float[]> currentFlow)
	{
		Map<String,float[]> ret = CopyFlow(currentFlow);
		Map<String,float[]> outf = CalcOutFlow(traffic,ret,Constants.TURN_PROBA);
		
		FlowAdd(ret,outf);
		return ret;
	}
	
	/***
	 * 将cost回馈
	 * 
	 * @param traffic - 交通结构图
	 * @param currentCost - 当前的cost
	 * @return 回馈的cost
	 */
	public static Map<String,float[]> Backward(TrafficGraph traffic,
			Map<String,float[]> currentCost)
	{
		return CalcOutFlow(traffic,currentCost, Constants.TURN_PROBA_REV);
	}
	
	private static void FlowAdd(Map<String,float[]> dst, Map<String,float[]>src)
	{
		FlowAdd(dst,src,1.0f);
	}
	
	/***
	 * 将src的流量加入dst
	 * 
	 * @param dst - 目标流量
	 * @param src - 源流量
	 * @param scale - 乘因子
	 */	
	private static void FlowAdd(Map<String,float[]> dst, Map<String,float[]>src, float scale)
	{//dst = dst+src
		for(Map.Entry<String,float[]> entry : src.entrySet())
		{
			String cid = entry.getKey();
			float[] flow = entry.getValue();
			
			if ( dst.containsKey(cid))
			{
				Utils.ArrayAdd(dst.get(cid), flow, scale);
			}
			else
			{
				dst.put(cid, flow.clone());
			}
		}
	}
	
	/***
	 * 将流量放大或缩小
	 * 
	 * @param dst - 目标
	 * @param scale - 乘因子
	 */
	private static void FlowScale(Map<String,float[]> dst, float scale)
	{
		for(Map.Entry<String,float[]> entry : dst.entrySet())
		{
			entry.setValue(Utils.ArrayScale(entry.getValue(), scale));
		}
	}
	
	/***
	 * 通过将流量前馈反馈算出每个节点四条边的权重，边的权重越大代表了将来可能会有大流量，
	 * 所以越不能往这个方向走
	 *  
	 * @param traffic 交通图结构
	 * @param currentTime 当前时间
	 * @param interval 前馈时间个数
	 * @return 每个节点对应4条边的权重
	 */
	public static Map<String,float[]> FlowPropagate(TrafficGraph traffic, int currentTime,
			int interval)
	{
		Map<String,float[]> currentFlow = traffic.getCurrentFlow();
		
		//forward
		List<Map<String,float[]> >fflows = new ArrayList<Map<String,float[]> >();
		for(int i=1;(i<interval)&&((currentTime+i)<Constants.MAX_TIME);i++)
		{
			//前馈
			Map<String,float[]> nextFlow = Forward(traffic,currentFlow);
			
			//加上每个路口突然出现的流量
			traffic.flowAdd(nextFlow, currentTime+i);
			fflows.add(nextFlow);
			currentFlow = nextFlow;
		}
		
		//backward
		Map<String,float[]> cost = new HashMap<String,float[]>();
		
		int revInd = fflows.size()-1;
		for(int i=1;(i<interval)&&((currentTime+i)<Constants.MAX_TIME);i++)
		{
			FlowAdd(cost, fflows.get(revInd));
			Map<String,float[]> tcost = Backward(traffic,cost);
			FlowAdd(cost,tcost);
			revInd--;
		}
		
		return cost;
	}
	
	public static int getInterval(int time) {
//		if(time < 30) return 4;
//		return 1;
		if(time < (float)Constants.MAX_TIME / 50) return Constants.MAX_LIGHT_INTERVAL;
		if(time < (float)Constants.MAX_TIME / 20) return Constants.MAX_LIGHT_INTERVAL - 1;
		if(time < (float)Constants.MAX_TIME / 10) return 2;
		return 1;
	}
	
	/***
	 * 检查公平原则
	 * 
	 * @param cid - 节点id
	 * @param time - 当前时间
	 * @param set - 当前设定
	 * @param traffic - 交通结构图
	 * @return 是否违反公平原则
	 */
	public static boolean IsMaxInterval(String cid, int time, int set, TrafficGraph traffic)
	{
		int[] history = traffic.crosses.get(cid).lightSettingHistory;
		
//		int interval = getInterval(time);
		int interval = Constants.MAX_LIGHT_INTERVAL;
		if (time % 120 < interval)
		{
			return false;
		}
		
		for(int i=time-1;i>=time-interval;i--)
		{
			if ( history[i] % 2 != set % 2)
			{
				return false;
			}
		}
		return true;
	}
	
	/***
	 * 根据cost确定红绿灯的设定
	 * 
	 * @param cost - 当前penalty
	 * @param estimateCost - 根据前馈和反馈计算出的边的权重
	 * @param setting - 各个节点的设定,output
	 * @param time - 当前时间
	 * @param traffic -交通结构图
	 * @return 总cost
	 */
	public static float Judge(Map<String,float[]> cost,
			Map<String,float[]> estimateCost, Map<String,Integer> setting,int time,
			TrafficGraph traffic)
	{
		float tcost = 0.0f;
		
		for(Map.Entry<String, float[]> entry : cost.entrySet())
		{
			String cid = entry.getKey();
			float[] flow = entry.getValue();
			
			int[] flowInt = Utils.ArrayFloat2Int(flow);
			CrossFlow cf = CalcCrossFlow(flowInt,Constants.TURN_PROBA);
			float[] costs = JudgeCost(flowInt,estimateCost.get(cid), cf);
			
			int csetting = -1;
			if ( costs[0] > costs[1])
			{
				csetting = 1;
			}
			else
			{
				csetting = 0;			
			}
			
			if ( IsMaxInterval(cid,time,csetting,traffic))
			{
				csetting = 1-csetting;
			}
			setting.put(cid, csetting);
			tcost += costs[csetting];
		}
		//SimpleLog.info("" + setting.get("tl32"));
		return tcost;
	}
	
	/***
	 * 
	 * @param traffic
	 * @param time
	 * @return
	 */
	public static float SolveSingle(TrafficGraph traffic, int time)
	{
		//估计交通图中每条边的权值
		Map<String,float[]> estimateCost = FlowPropagate(traffic,time,
				Constants.ESTIMATE_INTERVAL);
		
		//将权值缩小一定比例
		FlowScale(estimateCost,Constants.LAMBDA_1);
		
		//u
		Map<String,float[]> cost = traffic.getCurrentFlow();
		
		Map<String,Integer> setting = new HashMap<String,Integer>();
		
		//根据权值和流量计算红绿灯状态
		float tcost = Judge(cost, estimateCost, setting, time, traffic);
		
		traffic.setLight(setting, time);
		
		traffic.saveCurrentFlow();
		return tcost;
	}
	
	
//-----------------------------------------------------------------------------------------------------------
	public static boolean isInt(float f) {
		return Math.ceil(f) - f < eps;
	}
	public static boolean test(int mask, int wei) {
		if((mask & (1 << wei)) != 0)
			return true;
		return false;
	}
	public static int getMaxInterval(String cid, int time, int set, TrafficGraph traffic)
	{
		int[] history = traffic.crosses.get(cid).lightSettingHistory;
		
		if (time < Constants.MAX_LIGHT_INTERVAL)
		{
			return time;
		}
		
		for(int i=time-1;i>=0;i--)
		{
			if ( history[i] != set)
			{
				return time - i;
			}
		}
		return time + 1;
	}
	public static int getMaxInterval(String cid, int time, int set, int loc, TrafficGraph traffic)
	{
		if(test(set, loc)) return 0;
		int[] history = traffic.crosses.get(cid).lightSettingHistory;
		
		if (time < Constants.MAX_LIGHT_INTERVAL)
		{
			return time;
		}
		
		for(int i=time-1;;i--)
		{
			int mask = history[i];
			if(mask == 0) mask = 1495;
			else if(mask == 1) mask = 3770;
			else if(mask == 2) mask = 3575;
			else if(mask == 3) mask = 4030;
			if (test(mask, loc))
			{
				return time - i;
			}
			if(i % 120 == 0) break;
		}
		return time + 1;
	}
	
	public static Map<String,Integer> Copysetting(Map<String,Integer> setting)
	{
		Map<String,Integer> ret = new HashMap<String,Integer>();
		
		for(Map.Entry<String, Integer> entry: setting.entrySet())
		{
			ret.put(entry.getKey(), entry.getValue());
		}
		
		return ret;
	}
	public static int jud_probability(float pp[]) {
		pp[0] = (float)(1.0 / (pp[0] + 0.1));
		pp[1] = (float)(1.0 / (pp[1] + 0.1));
		double rp = pp[0] / (pp[0] + pp[1]);
		double randd = rand.nextDouble();
		if(randd > rp) return 1;
		else return 0;
	}
	public static int jud_stair(float pp[]) {
		if(pp[0] > pp[1]) return 1;
		else return 0;
	}
	public static int jud_random(float pp[]) {
		double randd = rand.nextDouble();
		if(randd > 0.5) return 1;
		else return 0;
	}
	
	/***
	 * 根据当前流量及红绿灯状态计算下一时间各个节点的流量
	 * 
	 * @param traffic - 交通结构图
	 * @param cflow - 当前流量
	 * @param turnProba - 转弯概率
	 * @param setting - 红绿灯状态
	 * @return 各个节点的流出量
	 */
	public static Map<String,float[]> CalcNxtFlow(TrafficGraph traffic, 
			Map<String,float[]> cflow, Map<String,Integer> setting, float[] turnProba)
	{
		Map<String,float[]> ret = new HashMap<String,float[]>();
		for(String cid : traffic.crosses.keySet())
		{
			ret.put(cid, new float[4]);
		}
		
		for(Map.Entry<String, float[]> entry : cflow.entrySet())
		{
			String cid = entry.getKey();
			int jud = setting.get(cid);
			float[] f = entry.getValue().clone();
			String[] nns = traffic.crosses.get(cid).neighbours;
			
			for(int i=0;i<f.length;i++)
			{
				String frm = cid;
				float[] tf = traffic.getFlow(nns[i], cid, f[i], turnProba.clone());	
				
				if(jud % 2 == i % 2) {
					for(int j=0;j<tf.length;j++)
					{
						int idst = (i+j+1) % 4;
						String dst = nns[idst];
						
						AddMapFlow(ret,frm,dst,traffic,(float)Math.ceil(tf[j]));
					}
					f[i] -= Utils.ArrayFloorSum(tf);
					AddMapFlow(ret, nns[i], cid, traffic, (float)Math.ceil(f[i]));
				} else {
					AddMapFlow(ret, nns[i], cid, traffic, (float)Math.ceil(f[i] - tf[2] - (jud > 1 ? tf[1] : 0)));
					AddMapFlow(ret, cid, nns[(i + 3) % 4], traffic, (float)Math.ceil(tf[2]));
					if(jud > 1)
						AddMapFlow(ret, cid, nns[(i + 2) % 4], traffic, (float)Math.ceil(tf[1]));
				}
			}
		}
		
		return ret;
	}
	
	public static void updateFlow(Map <String, Integer> setting, TrafficGraph traffic, int time) {
		Map <String, float[]> currentflow = traffic.getCurrentFlow();
		currentflow = CalcNxtFlow(traffic, currentflow, setting, Constants.TURN_PROBA);
		for(Map.Entry<String, float[]> entry : currentflow.entrySet()) {
			String id = entry.getKey();
			float[] flow1 = entry.getValue();
			int[] rflow = traffic.crosses.get(id).currentFlow;
			for(int i = 0; i < rflow.length; i++) 
				rflow[i] = (int)Math.ceil(flow1[i]);
		}
	}
	
	/***
	 * 给出单个路口的当前流量，计算罚时
	 * 
	 * @param id
	 * @param traffic
	 * @param flow
	 * @return 罚时(包含两个数，对应两种情况)
	 */
	public static float[] getSinglePen(String id, TrafficGraph traffic, float[] flow) {
		float[] ans = new float[2];
		String[] neigh = traffic.crosses.get(id).neighbours.clone(); 
		for(int cate = 0; cate < 2; cate++) {
			ans[cate] = flow[1 - cate] + flow[1 - cate + 2];
			ans[cate] += flow[cate] - Utils.ArraySum(traffic.getFlow(neigh[cate], id, flow[cate], Constants.TURN_PROBA.clone()));
			ans[cate] += flow[cate + 2] - Utils.ArraySum(traffic.getFlow(neigh[cate + 2], id, flow[cate + 2], Constants.TURN_PROBA.clone()));
			ans[cate] -= traffic.getFlow(neigh[1 - cate], id, flow[1 - cate], Constants.TURN_PROBA.clone())[2];
			ans[cate] -= traffic.getFlow(neigh[1 - cate + 2], id, flow[1 - cate + 2], Constants.TURN_PROBA.clone())[2];
		}
		return ans;
	}
	
	/***
	 * 给出单个路口的当前流量，计算罚时(考虑多出来的人头）
	 * 
	 * @param id
	 * @param traffic
	 * @param flow
	 * @return 罚时(包含两个数，对应两种情况)
	 */
	public static float[] getSinglePen_new(String id, TrafficGraph traffic, float[] flow, int time) {
		time = time % 120;
		int lefttime = 119 - time;
		float[] ans = new float[2];
		String[] neigh = traffic.crosses.get(id).neighbours.clone(); 
		for(int cate = 0; cate < 2; cate++) {
			ans[cate] = flow[1 - cate] + flow[1 - cate + 2];
			
			float[] temp = traffic.getFlow(neigh[cate], id, flow[cate], Constants.TURN_PROBA.clone());
			for(int i = 0; i < temp.length; i++) {
				if(!isInt(temp[i])) {
					ans[cate] += lefttime * Constants.LAMBDA_more;
					ans[cate]++;
				}
			}
			ans[cate] += flow[cate] - Utils.ArraySum(temp);
			
			temp = traffic.getFlow(neigh[cate + 2], id, flow[cate + 2], Constants.TURN_PROBA.clone());
			for(int i = 0; i < temp.length; i++) {
				if(!isInt(temp[i])) {
					ans[cate] += lefttime * Constants.LAMBDA_more;
					ans[cate]++;
				}
			}
			ans[cate] += flow[cate + 2] - Utils.ArraySum(temp);
			
			float tempfloat = traffic.getFlow(neigh[1 - cate], id, flow[1 - cate], Constants.TURN_PROBA.clone())[2]; 
			ans[cate] -= Math.floor(tempfloat);
			if(!isInt(tempfloat))
				ans[cate] += lefttime * Constants.LAMBDA_more;
			
			tempfloat = traffic.getFlow(neigh[1 - cate + 2], id, flow[1 - cate + 2], Constants.TURN_PROBA.clone())[2];
			ans[cate] -= Math.floor(tempfloat);
			if(!isInt(tempfloat))
				ans[cate] += lefttime * Constants.LAMBDA_more;
		}
		return ans;
	}
	
	/***
	 * 给出单个路口的当前流量，与路口具体红绿灯情况，计算罚时
	 * 
	 * @param id
	 * @param traffic
	 * @param flow
	 * @param setting
	 * @param time
	 * @return 罚时
	 */
	public static float getSinglePen(String id, TrafficGraph traffic, float[] flow, int setting, int time) {
		float ans = (float)0.0;
		String[] neigh = traffic.crosses.get(id).neighbours.clone();
		for(int i = 0; i < neigh.length; i++) if(neigh[i] == Constants.LIGHT_NONE) 
			for(int j = 0; j < 3; j++) if(test(setting, i * 3 + j))
				setting ^= 1 << (i * 3 + j);
			
		for(int i = 0; i < 4; i++) {
			float[] rflow = traffic.getFlow(neigh[i], id, flow[i], Constants.TURN_PROBA.clone());
			float temp = rflow[1];
			rflow[1] = rflow[2];
			rflow[2] = temp;

			float rans = flow[i];
			for(int j = 0; j < 3; j++) 
				if(test(setting, i * 3 + j)) 
					rans -= rflow[j];
			ans += rans;
		}
		
		double zeta = 0.5;
		for(int i = 0; i < 4; i++) if(test(setting, i * 3 + 2)) {
			int nxt1 = (i + 1) * 3 + 2, nxt2 = (i + 3) * 3 + 2;
			nxt1 %= 12; nxt2 %= 12;
			if((setting & ((1 << nxt1) | (1 << nxt2))) != 0) {
				ans += zeta * 0.5 * (flow[i] + flow[(i + 1) % 4] + flow[(i + 3) % 4]);
			}
		}
		for(int i = 0; i < 4; i++) {
			if(test(setting, i * 3) && test(setting, ((i + 1) * 3 + 2) % 12)) 
				ans += zeta * (flow[i] + flow[(i + 1) % 4]);
		}
		
		for(int i = 0; i < 12; i++) {
			int p = getMaxInterval(id, time, setting, i, traffic);
			if(p > Constants.MAX_LIGHT_INTERVAL) {
				ans += flow[i / 3] * Math.sqrt(p - Constants.MAX_LIGHT_INTERVAL);
//				return (float)1e10;
			}
		}
		
		return ans;
	}

	/***
	 * 给出单个路口的当前流量，与路口具体红绿灯情况，计算罚时(考虑多出来的人头)
	 * 
	 * @param id
	 * @param traffic
	 * @param flow
	 * @param setting
	 * @param time
	 * @return 罚时
	 */
	public static float getSinglePen_new(String id, TrafficGraph traffic, float[] flow, int setting, int time) {
		int lefttime = 119 - time % 120;
		float ans = (float)0.0;
		String[] neigh = traffic.crosses.get(id).neighbours.clone();
		for(int i = 0; i < neigh.length; i++) if(neigh[i] == Constants.LIGHT_NONE) 
			for(int j = 0; j < 3; j++) if(test(setting, i * 3 + j))
				setting ^= 1 << (i * 3 + j);
			
		for(int i = 0; i < 4; i++) {
			float[] rflow = traffic.getFlow(neigh[i], id, flow[i], Constants.TURN_PROBA.clone());
			float temp = rflow[1];
			rflow[1] = rflow[2];
			rflow[2] = temp;
			
			float rans = flow[i];
			boolean have = false;
			for(int j = 0; j < 3; j++) 
				if(test(setting, i * 3 + j)) { 
					rans -= rflow[j];
					if(!isInt(rflow[j])) {
						if(have == false) {
							have = true;
							continue;
						}
						rans++;
						rans += lefttime * Constants.LAMBDA_more;
					}
				}
			ans += rans;
		}
		
		double zeta = 0.5;
		for(int i = 0; i < 4; i++) if(test(setting, i * 3 + 2)) {
			int nxt1 = (i + 1) * 3 + 2, nxt2 = (i + 3) * 3 + 2;
			nxt1 %= 12; nxt2 %= 12;
			if((setting & ((1 << nxt1) | (1 << nxt2))) != 0) {
				ans += zeta * 0.5 * (flow[i] + flow[(i + 1) % 4] + flow[(i + 3) % 4]);
			}
		}
		for(int i = 0; i < 4; i++) {
			if(test(setting, i * 3) && test(setting, ((i + 1) * 3 + 2) % 12)) 
				ans += zeta * (flow[i] + flow[(i + 1) % 4]);
		}
		
		for(int i = 0; i < 12; i++) {
			int p = getMaxInterval(id, time, setting, i, traffic);
			if(p > Constants.MAX_LIGHT_INTERVAL) {
				ans += Math.ceil(flow[i / 3] * Math.sqrt(p - Constants.MAX_LIGHT_INTERVAL));
//				return (float)1e10;
			}
		}
		
		return ans;
	}
	
	/***
	 * 计算未来ESTIMATE_INTERVAL单位时间内罚时，未加入随机因素
	 * @param setting
	 * @param traffic
	 * @param time
	 * @return 罚时
	 */
	public static float cntpen(Map<String,Integer> setting, TrafficGraph traffic, int time) {
		float ans = 0;
		Map <String,float[]> currentflow = traffic.getCurrentFlow();
		Map<String,Integer> rsetting = Copysetting(setting);
		
		for(int rt = time; rt < time + Constants.ESTIMATE_INTERVAL && rt % 120 == time % 120; rt++) {
			for(Map.Entry<String,float[]> entry : currentflow.entrySet()) {
				float[] flow = entry.getValue();
				float[] rpen = getSinglePen(entry.getKey(), traffic, flow);
				if(rsetting.get(entry.getKey()) == 0) {
					ans += rpen[0];
				} else if(rsetting.get(entry.getKey()) == 1){
					ans += rpen[1];
				} else 
					ans += Utils.ArraySum(flow) * 0.75;
			}
			
			traffic.setLight(rsetting, rt);
			currentflow = CalcNxtFlow(traffic, currentflow, rsetting, Constants.TURN_PROBA);
			traffic.flowAdd(currentflow, rt + 1, Constants.LAMBDA_3);
			
			for(Map.Entry<String,float[]> entry : currentflow.entrySet()) {
				float[] flow = entry.getValue();
				int rval = 0;
				
//				float radd0 = 0, radd1 = 0;
//				radd0 = (float)getMaxInterval(entry.getKey(),rt,0,traffic);
//				radd1 = (float)getMaxInterval(entry.getKey(),rt,1,traffic);
//				if(radd0 > Constants.MAX_LIGHT_INTERVAL) radd0 = (float)Math.sqrt(radd0 - Constants.MAX_LIGHT_INTERVAL);
//				else radd0 = 0;
//				if(radd1 > Constants.MAX_LIGHT_INTERVAL) radd1 = (float)Math.sqrt(radd1 - Constants.MAX_LIGHT_INTERVAL);
//				else radd1 = 0;
//				
//				if(flow[0] + flow[2] + (flow[1] + flow[3]) * radd0 > (flow[0] + flow[2]) * radd1 + flow[1] + flow[3])
//					rval = 1;
				
				float[] pp = getSinglePen(entry.getKey(), traffic, flow);
				rval = jud_stair(pp);
				if ( IsMaxInterval(entry.getKey(),rt,rval,traffic) )
					rval = 1 - rval;
				
//				TrafficCrossroad t1 = traffic.crosses.get(entry.getKey());
//				boolean flag = false;
//				for(int i = 0; i < 4; i++) if(t1.neighbours[i] == Constants.LIGHT_NONE)
//					flag = true;
//				if(flag) continue;
//				if(pp[rval] > Utils.ArraySum(flow) * 0.75) {
//					pp[rval] = (float)(Utils.ArraySum(flow) * 0.75);
//					rval += 2;
//				}
				
				rsetting.put(entry.getKey(), rval);
			}
		}
		return ans;
	}
	
	/***
	 * 计算未来ESTIMATE_INTERVAL单位时间内罚时，加入随机因素
	 * 
	 * @param setting
	 * @param traffic
	 * @param time
	 * @return 罚时
	 */
	public static float cntpen1(Map<String,Integer> setting, TrafficGraph traffic, int time) {
		float ans = 0;
		Map <String,float[]> currentflow = traffic.getCurrentFlow();
		Map<String,Integer> rsetting = Copysetting(setting);
		
		for(int rt = time; rt < time + Constants.ESTIMATE_INTERVAL && rt % 120 == time % 120; rt++) {
			for(Map.Entry<String,float[]> entry : currentflow.entrySet()) {
				float[] flow = entry.getValue();
				float[] rpen = getSinglePen(entry.getKey(), traffic, flow);
				if(rsetting.get(entry.getKey()) == 0) {
					ans += rpen[0];
				} else if(rsetting.get(entry.getKey()) == 1){
					ans += rpen[1];
				} else 
					ans += Utils.ArraySum(flow) * 0.75;
			}
			
			traffic.setLight(rsetting, rt);
			currentflow = CalcNxtFlow(traffic, currentflow, rsetting, Constants.TURN_PROBA);
			traffic.flowAdd(currentflow, rt + 1, Constants.LAMBDA_3);
			
			for(Map.Entry<String,float[]> entry : currentflow.entrySet()) {
				float[] flow = entry.getValue();
				int rval = 0;
				float[] pp = getSinglePen(entry.getKey(), traffic, flow);
				rval = jud_probability(pp);
				if ( IsMaxInterval(entry.getKey(),rt,rval,traffic) )
					rval = 1 - rval;
				
//				TrafficCrossroad t1 = traffic.crosses.get(entry.getKey());
//				boolean flag = false;
//				for(int i = 0; i < 4; i++) if(t1.neighbours[i] == Constants.LIGHT_NONE)
//					flag = true;
//				if(flag) continue;
//				if(pp[rval] > Utils.ArraySum(flow) * 0.75) {
//					pp[rval] = (float)(Utils.ArraySum(flow) * 0.75);
//					rval += 2;
//				}
				
				rsetting.put(entry.getKey(), rval);
			}
		}
		return ans;
	}
	
	/***
	 * 
	 * @param traffic
	 * @param time
	 * @return
	 */
	public static float SolveSingle1(TrafficGraph traffic, int time) {
		Map<String,Integer> setting = new HashMap<String,Integer>();
		
		double rpen = 0.0;
		for(Map.Entry<String, TrafficCrossroad> entry : traffic.crosses.entrySet()) {
			String id = entry.getKey();
			float[] currentflow = new float[4];
			for(int i = 0; i < 4; i++)
				currentflow[i] = entry.getValue().currentFlow[i];
			int val = 0;
//			float radd0 = 0, radd1 = 0;
//			radd0 = (float)getMaxInterval(entry.getKey(),time,0,traffic);
//			radd1 = (float)getMaxInterval(entry.getKey(),time,1,traffic);
//			if(radd0 > Constants.MAX_LIGHT_INTERVAL) radd0 = (float)Math.sqrt(radd0 - Constants.MAX_LIGHT_INTERVAL);
//			else radd0 = 0;
//			if(radd1 > Constants.MAX_LIGHT_INTERVAL) radd1 = (float)Math.sqrt(radd1 - Constants.MAX_LIGHT_INTERVAL);
//			else radd1 = 0;
//			if(currentflow[0] + currentflow[2] + (currentflow[1] + currentflow[3]) * radd0 
//			   > (currentflow[0] + currentflow[2]) * radd1 + currentflow[1] + currentflow[3])
//				val = 1;
			
			
//			float[] pp = getSinglePen_new(id, traffic, currentflow, time);
			float[] pp = getSinglePen(id, traffic, currentflow);
			val = jud_stair(pp);

			setting.put(id, val);
		}
		
		Map<String,Integer> rsetting = Copysetting(setting), tsetting = Copysetting(setting);
		
		if(time < Constants.MAX_TIME / 10 + 2 - 100000) {
			
		for(Map.Entry<String, Integer> entry : rsetting.entrySet()) {
			int rval = entry.getValue();
			entry.setValue(1 - rval);
			float p1 = cntpen(rsetting, traffic, time);
			float p2 = cntpen(tsetting, traffic, time);
			if(p1 < p2) 
				tsetting.put(entry.getKey(), 1 - rval);
		}
		
		}
		
		for(Map.Entry<String, Integer> entry : tsetting.entrySet()) {
			int rval = entry.getValue();
//			float[] currentflow = new float[4];
//			for(int i = 0; i < 4; i++)
//				currentflow[i] = traffic.crosses.get(entry.getKey()).currentFlow[i];
//			float[] pp = getSinglePen(entry.getKey(), traffic, currentflow);
			if ( IsMaxInterval(entry.getKey(),time,rval,traffic)) {
//				if(pp[1 - rval] / (pp[0] + pp[1]) < 3.0 / 4)
//					entry.setValue(1 - rval);
//				else 
//					entry.setValue(2);
				rval = 1 - rval;
			}
			
//			float[] currentflow = new float[4];
//			for(int i = 0; i < 4; i++)
//				currentflow[i] = traffic.crosses.get(entry.getKey()).currentFlow[i];
//			float[] pp = getSinglePen(entry.getKey(), traffic, currentflow);
//			if(pp[rval] > Utils.ArraySum(currentflow) * 0.75)
//				rval += 2;
			
				
			entry.setValue(rval);
			
			//debug--------------------------------------------------------------------------------
			String id = entry.getKey();
			float[] currentflow = new float[4];
			for(int i = 0; i < 4; i++)
				currentflow[i] = traffic.crosses.get(id).currentFlow[i];
			float[] pp = getSinglePen(id, traffic, currentflow);
			rpen += pp[rval];
			//debug--------------------------------------------------------------------------------
		}
		
		
		traffic.pen += rpen;
		traffic.setLight(tsetting, time);
//		updateFlow(tsetting, traffic, time);
		traffic.saveCurrentFlow();
		return (float)0.0;
	}
	
	/***
	 * 方法1： 模拟退火，未加入随机因素
	 * 
	 * @param traffic
	 * @param time
	 * @return
	 */
	public static float SolveSingle2(TrafficGraph traffic, int time) {
		Map<String,Integer> setting = new HashMap<String,Integer>();
		for(Map.Entry<String, TrafficCrossroad> entry : traffic.crosses.entrySet()) {
			String id = entry.getKey();
			float[] currentflow = new float[4];
			for(int i = 0; i < 4; i++)
				currentflow[i] = entry.getValue().currentFlow[i];
			int val = 0;
			float[] pp = getSinglePen(id, traffic, currentflow);
			val = jud_probability(pp);
			if (IsMaxInterval(entry.getKey(),time,val,traffic)) {
				val = 1 - val;
			}

			setting.put(id, val);
		}
		
		if(time == 0) {                                                                         // can mend
			
			List <Map<String,Integer>> group = new ArrayList<Map<String,Integer>>();
			double[] pena = new double[Constants.GROUP_CNT + 1];
			double bestpen = cntpen1(setting, traffic, time);
			Map<String,Integer> bestsetting = Copysetting(setting);
			
			group.add(setting);
			pena[0] = bestpen;
			
			for(int rid = 1; rid < Constants.GROUP_CNT; rid++) {
				Map<String,Integer> tsetting = new HashMap<String,Integer>();
				group.add(tsetting);
			}
			for(Map.Entry<String, TrafficCrossroad> entry : traffic.crosses.entrySet()) {
				String id = entry.getKey();
				float[] currentflow = new float[4];
				for(int i = 0; i < 4; i++)
					currentflow[i] = entry.getValue().currentFlow[i];

				float[] pp = getSinglePen(id, traffic, currentflow);
				pp[0] = (float)(1.0 / (pp[0] + 0.1));
				pp[1] = (float)(1.0 / (pp[1] + 0.1));
//				double rp = pp[0] / (pp[0] + pp[1]);                                             // can mend
				double rp = 0.5;
				for(int i = 1; i < Constants.GROUP_CNT; i++) {
					int val = 0;
					double randd = rand.nextDouble();
					if(randd > rp) val = 1;
					if (IsMaxInterval(entry.getKey(),time,val,traffic)) 
						val = 1 - val;
					group.get(i).put(id,  val);
				}
			}
			for(int rid = 1; rid < Constants.GROUP_CNT; rid++) {
				pena[rid] = cntpen1(group.get(rid), traffic, time);
				if(pena[rid] < bestpen) {
					bestpen = pena[rid];
					bestsetting = Copysetting(group.get(rid));
				}
			}
			
			for(int i = 0; i < Constants.STOP_CNT; i++) {
				int id1 = Math.abs(rand.nextInt()) % Constants.GROUP_CNT;
				int id2 = Math.abs(rand.nextInt()) % Constants.GROUP_CNT;
				while(id2 == id1) 
					id2 = Math.abs(rand.nextInt()) % Constants.GROUP_CNT;
				
				Map <String, Integer> son1 = new HashMap <String, Integer>(); 
				Map <String, Integer> son2 = new HashMap <String, Integer>();                       // can remove
				for(Map.Entry<String, Integer> entry : group.get(id1).entrySet()) { 
					String id = entry.getKey();
					double p = rand.nextDouble();
					if(p > 0.5) {
						son1.put(id, group.get(id1).get(id));
					} else {
						son1.put(id, group.get(id2).get(id));						
					}
					p = rand.nextDouble();
					if(p > 0.5) {
						son2.put(id, group.get(id1).get(id));
					} else {
						son2.put(id, group.get(id2).get(id));						
					}
				} 
				double pen1 = cntpen(son1, traffic, time);
				double pen2 = cntpen(son2, traffic, time);
				if(Math.min(pen1, pen2) < Math.min(pena[id1], pena[id2])) {
					double p = rand.nextDouble();
					if(p <= Constants.p) {
						group.set(id1, son1);
						group.set(id2, son2);
						pena[id1] = pen1;
						pena[id2] = pen2;
					}
				} else {
					double p = rand.nextDouble();
					if(p >= Constants.p) {
						group.set(id1, son1);
						group.set(id2, son2);
						pena[id1] = pen1;
						pena[id2] = pen2;
					}
				}
				if(pen1 < bestpen) {
					i = -1;
					bestpen = pen1;
					bestsetting = Copysetting(son1);
				}
				if(pen2 < bestpen) {
					i = -1;
					bestpen = pen2;
					bestsetting = Copysetting(son2);
				}
			}
			
			setting = bestsetting;
		}
		
		traffic.setLight(setting, time);
//		updateFlow(setting, traffic, time);
		traffic.saveCurrentFlow();
		
		
		return (float)0.0;
	}
	
	/***
	 * 方法2： 遗传
	 * 
	 * @param traffic
	 * @param time
	 * @return
	 */
	public static float SolveSingle3(TrafficGraph traffic, int time) {
		Map<String,Integer> setting = new HashMap<String,Integer>();
		for(Map.Entry<String, TrafficCrossroad> entry : traffic.crosses.entrySet()) {
			String id = entry.getKey();
			float[] currentflow = new float[4];
			for(int i = 0; i < 4; i++)
				currentflow[i] = entry.getValue().currentFlow[i];
			int val = 0;
			float[] pp = getSinglePen(id, traffic, currentflow);
			val = jud_probability(pp);

			setting.put(id, val);
		}
		
		Map<String,Integer> rsetting = Copysetting(setting), tsetting = Copysetting(setting);
		
		if(time % 120 == 0) {
			
		for(Map.Entry<String, Integer> entry : rsetting.entrySet()) {
			int rval = entry.getValue();
			entry.setValue(1 - rval);
			float p1 = cntpen1(rsetting, traffic, time);
			float p2 = cntpen1(tsetting, traffic, time);
			if(p1 < p2) 
				tsetting.put(entry.getKey(), 1 - rval);
		}
		
		}
		
		for(Map.Entry<String, Integer> entry : tsetting.entrySet()) {
			int rval = entry.getValue();
			if (IsMaxInterval(entry.getKey(),time,rval,traffic)) {
				rval = 1 - rval;
			}
			
//			TrafficCrossroad t1 = traffic.crosses.get(entry.getKey());
//			boolean flag = false;
//			for(int i = 0; i < 4; i++) if(t1.neighbours[i] == Constants.LIGHT_NONE)
//				flag = true;
//			if(flag) continue;
//			float[] currentflow = new float[4];
//			for(int i = 0; i < 4; i++)
//				currentflow[i] = traffic.crosses.get(entry.getKey()).currentFlow[i];
//			float[] pp = getSinglePen(entry.getKey(), traffic, currentflow);
//			if(pp[rval] > Utils.ArraySum(currentflow) * 0.75)
//				rval += 2;
			
				
			entry.setValue(rval);
		}
		
		traffic.setLight(tsetting, time);
//		updateFlow(tsetting, traffic, time);
		traffic.saveCurrentFlow();
		return (float)0.0;
	}
	
	/***
	 * 方法3：模拟退火，决策部分加入随机化
	 * 
	 * @param traffic
	 * @param time
	 * @return
	 */
	public static float SolveSingle4(TrafficGraph traffic, int time) {
		double rpen = 0.0;
		Map<String,Integer> setting = new HashMap<String,Integer>();
		for(Map.Entry<String, TrafficCrossroad> entry : traffic.crosses.entrySet()) {
			String id = entry.getKey();
			float[] currentflow = new float[4];
			for(int i = 0; i < 4; i++)
				currentflow[i] = entry.getValue().currentFlow[i];
			int val = 1 << 12;
			float rp = (float)1e10;
			int mask = (1 << 12) - 1170 - 1;
			for(int i = mask; i >= 0; i = (i - 1) & mask) {
//				float pp = getSinglePen(id, traffic, currentflow, (i | (1 << 12) | 1170), time);
				float pp = getSinglePen_new(id, traffic, currentflow, (i | (1 << 12) | 1170), time);
				float[] rpp = new float[2];
				rpp[0] = rp; rpp[1] = pp;
				if(jud_stair(rpp) == 1) {
					val = (i | (1 << 12) | 1170);
					rp = pp;
				} 
				if(i == 0) break;
			}

			setting.put(id, val);
			
			//debug--------------------------------------------------------------------------------
			id = entry.getKey();
			for(int i = 0; i < 4; i++)
				currentflow[i] = traffic.crosses.get(id).currentFlow[i];
			rpen += getSinglePen(id, traffic, currentflow, val, time);
			//debug--------------------------------------------------------------------------------
		}
		
		Map<String,Integer> rsetting = Copysetting(setting), tsetting = Copysetting(setting);
		
//		if(time % 1 == 0) {
//			
//		for(Map.Entry<String, Integer> entry : rsetting.entrySet()) {
//			int rval = entry.getValue();
//			entry.setValue(1 - rval);
//			float p1 = cntpen1(rsetting, traffic, time);
//			float p2 = cntpen1(tsetting, traffic, time);
//			if(p1 < p2) 
//				tsetting.put(entry.getKey(), 1 - rval);
//		}
//		
//		}
		
//		for(Map.Entry<String, Integer> entry : tsetting.entrySet()) {
//			int rval = entry.getValue();
//			if (IsMaxInterval(entry.getKey(),time,rval,traffic)) {
//				rval = 1 - rval;
//			}
//			
////			TrafficCrossroad t1 = traffic.crosses.get(entry.getKey());
////			boolean flag = false;
////			for(int i = 0; i < 4; i++) if(t1.neighbours[i] == Constants.LIGHT_NONE)
////				flag = true;
////			if(flag) continue;
////			float[] currentflow = new float[4];
////			for(int i = 0; i < 4; i++)
////				currentflow[i] = traffic.crosses.get(entry.getKey()).currentFlow[i];
////			float[] pp = getSinglePen(entry.getKey(), traffic, currentflow);
////			if(pp[rval] > Utils.ArraySum(currentflow) * 0.75)
////				rval += 2;
//			
//				
//			entry.setValue(rval);
//		}
		
		traffic.pen += rpen;
		traffic.setLight(tsetting, time);
//		updateFlow(tsetting, traffic, time);
		traffic.saveCurrentFlow();
		return (float)0.0;
	}
	
	/***
	 * 方法4：每个路口灯都枚举所有可能，未加遗传与模拟退火
	 * 
	 * @param traffic
	 * @param flow
	 * @param time
	 * @return
	 */
	public static float Solve(TrafficGraph traffic, Map<String,int[]> flow, int time)
	{
		traffic.setCurrentFlow(flow, time);
		return SolveSingle4(traffic,time);
	}
}
