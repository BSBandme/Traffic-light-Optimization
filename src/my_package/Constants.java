package my_package;


public class Constants 
{
	public final static int MAX_TIME = 1681; //��ʱ��
	public final static int ESTIMATE_INTERVAL = 10; //Ԥ��ʱ���
	public final static float LAMBDA_1 =  1f; //Ȩ��
	public final static float LAMBDA_2 = -0.5f; //Ȩ��
	public final static float LAMBDA_3 = 0.5f; //Ȩ��
	public final static float LAMBDA_more = 0.08f; //Ȩ��
	
	public final static float[] TURN_PROBA = {0.1f,0.8f,0.1f}; //ת����ʣ�������
	public final static float[] TURN_PROBA_REV = {0.1f, 0.8f, 0.1f}; //�෴��ת����ʣ�������
	
	public final static float[] MAX_THROUGH = {2.0f,16.0f,2.0f}; //���ͨ������������
	public final static String LIGHT_NONE = "#";
	
	
	public final static String FILENAME_TRAFFIC="TrafficLightTable.txt";
	public final static String FILENAME_FLOW_ADD="flow0901.txt";
	public final static String FILENAME_Debug="debug_information.txt";
//	public final static String FILENAME_TRAFFIC="Tt.txt";
//	public final static String FILENAME_FLOW_ADD="flow1.txt";
	
	public final static int MAX_LIGHT_INTERVAL = 4;
	
	public final static int GROUP_CNT = 100; //��Ⱥ����
	public final static int STOP_CNT = 200; //��ֹ��������
	public final static double p = 0.9; //���ʺ�����ܸ���
	
	public final static boolean debug = true;
	
}
