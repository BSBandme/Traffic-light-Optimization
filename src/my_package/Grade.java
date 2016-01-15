package my_package;
 
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

//import com.google.common.collect.Lists;
//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;

public class Grade {

	//��Traffic_Light_Table.txt��ȡ����·�ں��̵�λ�ù�ϵ��Ϣ
	static Map<String, String[]> trafficLightTable = new ConcurrentHashMap<String, String[]>();
	
	//����·��ĳ��Сʱ�ľ�̬������Ͷ�̬������(���鳤��Ϊ120)
	static Map<String, Integer[]> staticFlowTable = new ConcurrentHashMap<String,Integer[]>();
	static Map<String, Integer[]> dynamicFlowTable = new ConcurrentHashMap<String,Integer[]>();
	
	//������ѡ�ֲ���:ĳ��Сʱĳ��ʱ��T(i)����·�ڵĳ�����
	static Map<String, Integer> currentFlows = new ConcurrentHashMap<String,Integer>();
	
	//������ѡ�ֲ���:ĳ��ʱ��T(i)����·�ڳ����� ת�����:[��ת,��ת,ֱ��]
	static Map<String, Double[]> turnRate = new ConcurrentHashMap<String,Double[]>();
	
	//����������ÿ��·��ͨ����level
	static Map<String,Integer> throughLevel = new ConcurrentHashMap<String,Integer>();
	
	//������ѡ�ֲ���:ĳ��ʱ��T(i)����·�ڳ����� ͨ����:[��ת,��ת,ֱ��]
	static Map<String, Integer[]> throughRate = new ConcurrentHashMap<String,Integer[]>();
	
	//ѡ�ִ��������ĳ��ʱ��T(i)����·�ں� �̵Ƶ�״̬:[��ת,��ת,ֱ��] (���Ϊ0���̵�Ϊ1,ȱʧ-1)
	static Map<String, Integer[]> currentStatus = new ConcurrentHashMap<String,Integer[]>();
	
	//ĳ��Сʱ�ں��̵Ƶ���ʷ��Ϣ
	static Map<String, ArrayList<Integer[]>> statusHistory = new ConcurrentHashMap<String,ArrayList<Integer[]>>();
	
	//14��Сʱ��penalty
			
	//����taskId��Ӧ��penalty
	public static Map<String, Double[]> penaltyMap =  new ConcurrentHashMap<String, Double[]>();
	
	//��ʼ��ÿ��ѡ�ֶ�Ӧ��penalty����
	public static void initPenalty(String taskId) {
		Double[] penalty = {0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0};
		penaltyMap.put(taskId, penalty);
	}
	
	//��trafficLightTable��ȡ���̵�λ����Ϣ
	public static void setLightInfo(String taskId, Map<String, String[]> trafficLightMap){
		for (String aKey : trafficLightMap.keySet()) {
			String[] keyStrings = aKey.split("-");
			String task_key = keyStrings[0] + "-" + keyStrings[1] + "-" + taskId;
			trafficLightTable.put(task_key,trafficLightMap.get(aKey));
		}

	}
	
	//����throughLevel
	public static void setLevelInfo(String taskId, Map<String, Integer>levleMap) {
		for (String aKey : levleMap.keySet()) {
			String[] keyStrings = aKey.split("-");
			String task_key = keyStrings[0] + "-" + keyStrings[1] + "-" + taskId;
			throughLevel.put(task_key, levleMap.get(aKey));
		}
	}
	
	// ��i��Сʱ״̬��ʼ��
	public static void hourInit(String taskId, Map<String, Integer[]> trafficFlowMap,int i) throws NumberFormatException, IOException {
		
		for (String aKey : trafficFlowMap.keySet()) {
			String[] keyStrings = aKey.split("-");
			String taks_key = keyStrings[0] + "-" + keyStrings[1] + "-" + taskId;
			
			  Integer[] values = new Integer[120];
			  for (int i1 = 0; i1 < values.length; i1++) {
				 values[i1] = trafficFlowMap.get(aKey)[i*120 + i1];			
			}
			staticFlowTable.put(taks_key,values);
			dynamicFlowTable.put(taks_key,values.clone());
		}
		
		//���������޸�
		//��ʼ��turnRate:[0.2,0.2,0.6]��throughRate��statusHistory
		for(String key : trafficLightTable.keySet()){

			if(!taskId.equals(key.split("-")[2])){
				continue;
			}
			//���������޸� ʮ��·��ת�����
			Double[] initTurnRate = {0.2,0.2,0.6};			
			//����·��,����ת����
			if (trafficLightTable.get(key)[0].equals("#")) {
				initTurnRate[0] = 0.0;
				initTurnRate[1] = 0.4;				
			}else if (trafficLightTable.get(key)[1].equals("#")) {
				//����·��,����ת����
				initTurnRate[0] = 0.4;
				initTurnRate[1] = 0.0;
			}else if (trafficLightTable.get(key)[2].equals("#")) {
				//����·��,��ֱ�з���
				initTurnRate[0] = 0.5;
				initTurnRate[1] = 0.5;
				initTurnRate[2] = 0.0;
			}
			
			//����ͨ���ʲ����޸�
			Integer[] initThroughRate = new Integer[3];
			for (i=0;i<initThroughRate.length;i++ ) {
				initThroughRate[i] = (int)((20-5*throughLevel.get(key))*initTurnRate[i]);
			}			
			
			ArrayList<Integer[]> initStatusHistory = new ArrayList<Integer[]>();
			for (int j = 0; j < 120; j++) {				
				Integer[] aStatus = {0,0,0};
				if ("#".equals(trafficLightTable.get(key)[0])) {
					aStatus[0] = -1;
				}else if ("#".equals(trafficLightTable.get(key)[1])) {
					aStatus[1] = -1;
				}else if ("#".equals(trafficLightTable.get(key)[2])) {
					aStatus[2] = -1;
				}
			
				initStatusHistory.add(aStatus);
			}
			
			turnRate.put(key, initTurnRate);
			throughRate.put(key, initThroughRate);
			statusHistory.put(key, initStatusHistory);
		}
	}
	
	//ѡ�ִ����������iʱ��currentStatus,���µ�statusHistory
	public static void setCurrentStatus(Map<String, Integer[]> aStatus,int i) {
				
		for(String key :aStatus.keySet()){
			if (statusHistory.containsKey(key)) {
				for (int j = 0; j < 3; j++) {
					if ("#".equals(trafficLightTable.get(key)[j])) {
						//���·�ڲ�ͨ��ǿ�ƽ�������Ϊ-1
						statusHistory.get(key).get(i)[j] = -1;
					} else {
						//���·����ͨ�ģ�������ѡ�ֽ�������Ϊ-1
						statusHistory.get(key).get(i)[j] = Math.max(0, aStatus.get(key)[j]);
					}
				}
				
			}
		}
	}
	
	//����statusHistory(i)��dynamicFlowTable(i)����T(i)ʱ��ĳ·������������
	public static int computeStay(String key, int i) {
		
		//��ת��תֱ��ʵ���ܹ�ͨ���ĳ�����
		int leftThrough,rightThrough,straightThrough;
		leftThrough=rightThrough=straightThrough = 0;
		//�̵�ʱͨ���ʲ���Ч
		if (statusHistory.get(key).get(i)[0]==1) {
			leftThrough = throughRate.get(key)[0];
		}
		if (statusHistory.get(key).get(i)[1]==1) {
			rightThrough = throughRate.get(key)[1];
		}
		if (statusHistory.get(key).get(i)[2]==1) {
			straightThrough = throughRate.get(key)[2];
		}
		
		//dynamicFlow(i)
		int iFlow = dynamicFlowTable.get(key)[i];
		//��ת��תֱ�е���������
		int leftStay,rightStay,straightStay;
		leftStay = rightStay = straightStay = 0;
		
		double leftRate = turnRate.get(key)[0];
		double rightRate = turnRate.get(key)[1];
		double straightRate = turnRate.get(key)[2];

		//�����޸ģ����������������
		//����·������ת
		if(leftRate==0.0){
	        leftStay = 0;
	        rightStay = Math.max(0, (int)Math.floor(iFlow*rightRate) - rightThrough);
	        straightStay = Math.max(0, iFlow-(int)Math.floor(iFlow*rightRate)-straightThrough);   
		}
		//����·������ת
		else if(rightRate==0.0){
	        leftStay = Math.max(0, (int)Math.floor(iFlow*leftRate) - leftThrough);
	        rightStay = 0;
	        straightStay = Math.max(0, iFlow-(int)Math.floor(iFlow*leftRate)-straightThrough);
		}
		//����·����ֱ��
		else if(straightRate==0.0){
	        leftStay = Math.max(0, (int)Math.floor(iFlow*leftRate) - leftThrough);
		    rightStay = Math.max(0, iFlow-(int)Math.floor(iFlow*leftRate) - rightThrough);
		    straightStay = 0;
		}
		//ʮ��·��
		else{
	        leftStay = Math.max(0, (int)Math.floor(iFlow*leftRate) - leftThrough);
		    rightStay = Math.max(0, (int)Math.floor(iFlow*rightRate) - rightThrough);
		    straightStay = Math.max(0, iFlow-(int)Math.floor(iFlow*leftRate)- (int)Math.floor(iFlow*rightRate)-straightThrough);
		}
		
		//LOGGER.error("key="+key+",i="+i+",leftStay="+leftStay+",rightStay="+rightStay+",straightStay="+straightStay);
		//����T(i)ʱ������������
		//System.out.println("allStay===" +(leftStay + rightStay + straightStay));
		return (leftStay + rightStay + straightStay);
		
	}
	
	/*���ݺ��̵�λ����Ϣ��T(i-1)ʱ�εģ������������к��̵�״̬������ת����ʡ�����ͨ���� 
     *����T(i)ʱ�̳�����
     *dynamicFlowTable(i)=LAMADA*staticFlowTable(i) + G(trafficLightTable,statusHistory(i-1),dynamicFlowTable(i-1),turnRate(i-1),throughRate(i-1))
     */
	public static void updateDynamicFlowTable(String taskId, int i) {
		
		for(String key :dynamicFlowTable.keySet()){
    		
			String tId = key.split("-")[2];
    		
			if (tId.equals(taskId)) {
				if (i==0) {
		    		dynamicFlowTable.get(key)[i] =  staticFlowTable.get(key)[i];					
				} else {
					
					//�۲�ֵϵ��LAMADA
					double LAMADA = 0.5;
					//����flow,����ϵ��LAMADA*staticFlow(i)
					dynamicFlowTable.get(key)[i] = (int)(Math.floor(LAMADA*staticFlowTable.get(key)[i]));
					
					//���statusHistory(i-1)���� dynamicFlow(i-1)�� ��ת����+��ת����+ֱ������						
					int allStay = 0;
					//�Ż������ú���ʵ��
					allStay = computeStay(key, i-1);											
					//���£�������������
					dynamicFlowTable.get(key)[i] += allStay;
										
					//����FromID������·������ĳ���
					int flowIn1,flowIn2,flowIn3;
					flowIn1 = flowIn2 = flowIn3 = 0;
					
					//����·��ID
					String[] keyStrings = key.split("-");
					String antiKey = keyStrings[1] + "-" + keyStrings[0] + "-" + taskId;
					//����·��������trafficLightTable�в�������
					if (trafficLightTable.containsKey(antiKey)) {
						//���복������ԴID
						String antiLeftID = trafficLightTable.get(antiKey)[0];
						String antiRightID = trafficLightTable.get(antiKey)[1];
						String antiStraightID = trafficLightTable.get(antiKey)[2];
							
						String antiLeftKey = keyStrings[1]  + "-" + antiLeftID + "-" + keyStrings[2];
						String antiRightKey = keyStrings[1] + "-" + antiRightID + "-" + keyStrings[2];
						String antiStraightKey = keyStrings[1] + "-" + antiStraightID + "-" + keyStrings[2];							
							
						//�� antiLeftID ��ת����
						if (!(trafficLightTable.get(antiLeftKey)==null)&&(statusHistory.get(antiLeftKey).get(i-1)[1]==1)) {
							//��ֱ�ж���·�����,��ת=all-��ת
							if(statusHistory.get(antiLeftKey).get(i-1)[2] == -1){
								flowIn1 = Math.min(throughRate.get(antiLeftKey)[1], 
									dynamicFlowTable.get(antiLeftKey)[i-1] - (int)Math.floor(dynamicFlowTable.get(antiLeftKey)[i-1]*turnRate.get(antiLeftKey)[0]));
							}else{//����ת����·����ʮ��·�����
								flowIn1 = Math.min(throughRate.get(antiLeftKey)[1], 
									(int)Math.floor(dynamicFlowTable.get(antiLeftKey)[i-1]*turnRate.get(antiLeftKey)[1]));
							}

						}
						
						//�� antiRightID ��ת����
						if (!(trafficLightTable.get(antiRightKey)==null)&&(statusHistory.get(antiRightKey).get(i-1)[0]==1)) {
							flowIn2 = Math.min(throughRate.get(antiRightKey)[0], 
								(int)Math.floor(dynamicFlowTable.get(antiRightKey)[i-1]*turnRate.get(antiRightKey)[0]));
						}

						//�� antiStraightKey ֱ������
						if(!(trafficLightTable.get(antiStraightKey)==null)&&(statusHistory.get(antiStraightKey).get(i-1)[2]==1)){
							if (statusHistory.get(antiStraightKey).get(i-1)[0] == -1) {//����ת����·��
								flowIn3 = Math.min(throughRate.get(antiStraightKey)[2],
									dynamicFlowTable.get(antiStraightKey)[i-1] 
									- (int)Math.floor(dynamicFlowTable.get(antiStraightKey)[i-1]*turnRate.get(antiStraightKey)[1]));								
							}else if (statusHistory.get(antiStraightKey).get(i-1)[1] == -1) {//����ת����·��
								flowIn3 = Math.min(throughRate.get(antiStraightKey)[2],
									dynamicFlowTable.get(antiStraightKey)[i-1] 
									- (int)Math.floor(dynamicFlowTable.get(antiStraightKey)[i-1]*turnRate.get(antiStraightKey)[0]));								
							}else{//ʮ��·��
								flowIn3 = Math.min(throughRate.get(antiStraightKey)[2],
									dynamicFlowTable.get(antiStraightKey)[i-1] 
									- (int)Math.floor(dynamicFlowTable.get(antiStraightKey)[i-1]*turnRate.get(antiStraightKey)[0])
									- (int)Math.floor(dynamicFlowTable.get(antiStraightKey)[i-1]*turnRate.get(antiStraightKey)[1]));
							}
						}
					}					

					//���£��������복��
					dynamicFlowTable.get(key)[i] += flowIn1 + flowIn2 + flowIn3;
					//System.out.println("  " + key +"-" + i + " flow:" +dynamicFlowTable.get(key)[i] );
						
				}
			}
		}
			
	}

	//�� dynamicFlowTable ���� currentFlow ������ѡ��
	public static Map<String, Integer> getCurrentFlow(String taskId, int i){

			for(String key : dynamicFlowTable.keySet()){
				if (taskId.equals(key.split("-")[2])) {
					currentFlows.put(key, dynamicFlowTable.get(key)[i]);
				}
			}
			return currentFlows;
	}	

	//����currentStatus(i)���µ�k��Сʱpenalty
	public static void updatePenalty(String taskId,int i,int k){
		
		//ÿ��·��T(i)ʱ��penalty: ��ת����+��ת����+ֱ������;Υ����ͨ����۷�;Υ����ƽ��ԭ��۷�
		for(String key : dynamicFlowTable.keySet()){
			
			String[] lights = key.split("-");
			if (taskId.equals(lights[2])) {
				//���£�������������
				penaltyMap.get(taskId)[k] += computeStay(key, i);
				//���£����Ϻ��̵�Υ����ͨ����ĳͷ� a:ֱ�д�ֱֱ�гͷ� b:ֱ�д�ֱ��ת�ͷ�
				double a,b;
				a=b=0.0;			
				//��ͨΥ��ĳͷ�����
				double zeta =0.5;
				
				String leftKey = lights[0] + "-" + trafficLightTable.get(key)[0] + "-" + lights[2];
				String rightKey = lights[0] + "-" + trafficLightTable.get(key)[1] + "-" + lights[2];
				
				//��ֱ������ͬʱֱ��																				
				if (statusHistory.get(key).get(i)[2]==1 &&
					((statusHistory.containsKey(leftKey) && statusHistory.get(leftKey).get(i)[2]==1) 
					|| (statusHistory.containsKey(rightKey) && statusHistory.get(rightKey).get(i)[2]==1))) {
						
					a += zeta*dynamicFlowTable.get(key)[i];
						
					if (dynamicFlowTable.containsKey(leftKey)) {
							a += zeta*dynamicFlowTable.get(leftKey)[i];
					}
						
					if (dynamicFlowTable.containsKey(rightKey)) {
							a += zeta*dynamicFlowTable.get(rightKey)[i];
					}
						
				}
				
				//ֱ��ʱ��ֱ�����Ҳ಻����ת
				if (statusHistory.get(key).get(i)[2]==1 && statusHistory.containsKey(rightKey) && statusHistory.get(rightKey).get(i)[0]==1) {

						b += zeta*(dynamicFlowTable.get(rightKey)[i] + dynamicFlowTable.get(key)[i]);
				}
				
				//Υ��۷�
				penaltyMap.get(taskId)[k] += 0.5*a + b;

				//���£�����Υ����ƽԭ��۷� v*sqrt(r-4)
				if (i>3) {
					for (int j = 0; j < 3; j++) {
						if (statusHistory.get(key).get(i)[j]==0) {
							int waitTime = 1;
							int waitStart = i;
							//�޸�bug
							while ((waitStart>0) && statusHistory.get(key).get(waitStart-1)[j]==0) {
								waitTime += 1;
								waitStart -=1;
							}
							penaltyMap.get(taskId)[k] += (int)Math.ceil(dynamicFlowTable.get(key)[i]*Math.sqrt(Math.max(waitTime-4, 0)));
						}
					}
				}
				
			}
																													
		}
		//System.out.println(k + "th hour" + i + "th Penalty======" + penaltyMap.get(taskId)[k]);
		
	}	
	
	//���ĳ���û��Ļ�������
	public static void cleanCache(String taskId) {
		List<String> list = new ArrayList<String>();
		for(Map.Entry<String, String[]> entry : trafficLightTable.entrySet()) 
			list.add(entry.getKey());
		for(String key : list){
			String[] keyStrings = key.split("-");
			if (keyStrings[2].equals(taskId)) {
				trafficLightTable.remove(key);
				staticFlowTable.remove(key);
				dynamicFlowTable.remove(key);
				turnRate.remove(key);
				throughLevel.remove(key);
				throughRate.remove(key);
				statusHistory.remove(key);
			}
		}
		penaltyMap.remove(taskId);
	}
	
	//���ѡ�����յ÷�
	public static int getGrade(String taskId) {
		
		if (penaltyMap.containsKey(taskId)) {
			int score = 0;
			for (int i = 0; i < penaltyMap.get(taskId).length; i++) {
				score += penaltyMap.get(taskId)[i];	
		}			

            //�建������
			cleanCache(taskId);
            System.out.println("returnSumScore====" + score);
			return score;
		} else {
			//���쳣��ѡ�ֵ÷�Ϊ-1
			return -1;
		}			
	}
	
}




