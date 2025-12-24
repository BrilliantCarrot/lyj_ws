using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using System;
using PA_DronePack;
using UnityEngine.UI;

public class DroneAgent : Agent
{
    private PA_DroneController dcoScript;
    // 오브젝트 변수
    public GameObject Drone;
    private Rigidbody DroneAgent_Rigidbody;
    public GameObject goal; 
    public int numOfGoals = 50;
    public GameObject[] Goals = new GameObject[50];
    public GameObject[] Ranges = new GameObject[50];
    public GameObject targetRange;      // 목표점의 반경 객체
    private Material matDefault;
    private Material matDetection;
    // 에이전트, 목표점, 목표점 반경 위치 변수 및 배열 초기화
    private Transform agentTrans;
    private Transform DroneTrans;
    private Transform goalTrans;
    private Transform targetRangeTrans;  // 목표점의 위치 선언
    private Transform[] GoalsTrans = new Transform[50];
    private Transform[] rangesTrans = new Transform[50];
    private Vector3 droneInitPos;               // DroneTrans 형 position을 저장하는 Vector3 형 droneInitPos
    private Quaternion droneInitRot;
    float cylinderHeight = 0.5f;
    float cylinderRadius = 40f;

    float goalDiff;
    float[] goalDiffArray = new float[50];     // 거리 비교 goalDiff가 저장될 배열  
    float preDist;
    
    private Text goalNum;
    private Text statusText;
    private Text yText;
    private Text xText;
    private Text zText;
    private Text rewardText;
    bool hit = false;
    bool first = true;
    // int cnt = 0;

    void Start(){
        Debug.Log("Start 함수 실행");
        DroneTrans = Drone.transform;
        droneInitPos = DroneTrans.position; // Transform 형의 position을 저장하는 Vector3 형
        droneInitRot = DroneTrans.rotation;

        DroneAgent_Rigidbody = Drone.GetComponent<Rigidbody>();

        matDefault = Resources.Load("Materials/Default", typeof(Material)) as Material;
        matDetection = Resources.Load("Materials/Detection", typeof(Material)) as Material;

        goalNum = GameObject.Find("Text").GetComponent<Text>();
        statusText = GameObject.Find("Status").GetComponent<Text>();
        yText = GameObject.Find("Height").GetComponent<Text>();
        xText = GameObject.Find("X").GetComponent<Text>();
        zText = GameObject.Find("Z").GetComponent<Text>();
        rewardText = GameObject.Find("Reward").GetComponent<Text>();

        // 게임이 실행되면 목표점을 저장하고 해당 목표점들의 반경 표시 객체를 생성
        for(int i = 0; i<numOfGoals;i++){
            Goals[i] = GameObject.Find((i+1).ToString());
            GoalsTrans[i] = Goals[i].transform;

            GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cylinder.GetComponent<CapsuleCollider>().enabled = false;
            cylinder.transform.localScale = new Vector3(cylinderRadius,cylinderHeight,cylinderRadius);
            cylinder.transform.position = new Vector3(GoalsTrans[i].position.x, 0, GoalsTrans[i].position.z);
            cylinder.GetComponent<Renderer>().material = matDefault;
            Ranges[i] = cylinder;
            rangesTrans[i] = cylinder.transform;
        }
    }

    // 드론 에이전트의 초기 상태 정의
    public void DroneInit(){
        DroneAgent_Rigidbody.velocity = Vector3.zero;
        DroneAgent_Rigidbody.angularVelocity = Vector3.zero;
        DroneTrans.position = droneInitPos;
        DroneTrans.rotation = droneInitRot;
        Debug.Log("DroneInit, 에이전트 위치 이동됨: "+DroneTrans.position);
    }

    // 최근접 목표점 탐색 함수
    public (GameObject goal,GameObject targetRange) SearchGoal(){
        float closestDistanceSqr = Mathf.Infinity;
        int j = 0;
        for(int i = 0; i<numOfGoals;i++){
            Vector3 directionToTarget = (GoalsTrans[i].position - DroneTrans.position);
            float dSqrToTarget = directionToTarget.sqrMagnitude;
            if(dSqrToTarget < closestDistanceSqr){
                closestDistanceSqr = dSqrToTarget;
                goal = Goals[i];
                targetRange = Ranges[i];
                j = i;
            }
            // goalDiffArray 초기화하여 거리 정보 새로 업데이트 가능하도록 수정
            goalDiffArray[i] = float.NaN;
        }
        Debug.Log("SearchGoal 함수 내 "+(j+1)+"번째 후보지가 목표점 지정됨");
        return (goal,targetRange);
    }

    // 목표점의 반경 targetRange의 Material을 변경
    public void ChangeColor(GameObject targetRange, Material matDetection){
        targetRange.GetComponent<Renderer>().material = matDetection;
    }

    // 목표점을 찾은 후 드론의 초기 위치를 해당 목표점으로 업데이트
    public void MoveDronePos(){
        Debug.Log("드론 위치를 탐색에 성공한 목표점으로 이동시킵니다");
        droneInitPos = goalTrans.position;
        DroneInit();
    }

    // 도달한 목표점과 영역이 겹치는 다른 목표점 삭제
    // 반경이 겹치는 목표점들을 제거하면서 동시에 새 목표점 탐색도 수행
    public void RemoveGoals(){
        for(int i = 0; i<numOfGoals;i++){
            goalDiff = Vector3.Magnitude(goalTrans.position - GoalsTrans[i].position);
            if(goalDiff != 0){
                goalDiffArray[i] = goalDiff;
            }
            Debug.Log("목표점 " + (i+1) + "의 거리차는 " + goalDiff +"입니다");
            // 반경이 서로 겹치는 범위 내에 존재하며 배열내 같은 목표점끼리 비교하는것이 아니라면 목표점을 삭제
            if(goalDiff < 40 && goalDiff != 0){
                GoalsTrans[i].position = new Vector3(160,20,160);
                rangesTrans[i].position = new Vector3(160,1,160);
                Debug.Log((i+1) + "번째 후보지가 제거되었습니다");
            }
        }
        goalTrans.position = new Vector3(160,20,160);   // 마지막으로 골도 제거(이동)
        var (goal,targetRange) = SearchGoal();
        goalTrans = goal.transform;
        targetRangeTrans = targetRange.transform;
        preDist = Vector3.Magnitude(goalTrans.position - agentTrans.position);
    }

    public override void Initialize()
    {
        dcoScript = gameObject.GetComponent<PA_DroneController>();
        agentTrans = gameObject.transform;
        Academy.Instance.AgentPreStep += WaitTimeInference;
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("새 에피소드 시작");
        // 첫번째로 시작되는 에피소드에서 목표점을 탐색
        if(first == true){
            var (goal,targetRange) = SearchGoal();
            first = false;
            statusText.text = "첫 번째 에피소드";
        }

        Debug.Log(goal+" 이(가) 목표점입니다.");
        goalTrans = goal.transform;
        targetRangeTrans = targetRange.transform;
        DroneInit();
        preDist = Vector3.Magnitude(goalTrans.position - agentTrans.position);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
            AddReward(-0.01f);
            var actions = actionBuffers.ContinuousActions;
            float moveX = Mathf.Clamp(actions[0], -1, 1f);
            float moveY = Mathf.Clamp(actions[1], -1, 1f);
            float moveZ = Mathf.Clamp(actions[2], -1, 1f);
            dcoScript.DriveInput(moveX);
            dcoScript.StrafeInput(moveY);
            dcoScript.LiftInput(moveZ);

            float distance = Vector3.Magnitude(goalTrans.position - agentTrans.position);

            if(distance < 2f){      // 드론이 목표점 위치로 이동에 성공했다면
                hit = true;
                SetReward(50f);     // 보상을 부여하고
                ChangeColor(targetRange,matDetection);  // 시각화를 위해 색깔을 변경하고
                MoveDronePos();     // 드론의 초기 위치를 이동하며
                RemoveGoals();      // 목표점 근처 겹치는 반경의 목표점들을 제거
                EndEpisode();
                statusText.text = "목표점 도달 성공";
            }

            if(distance > 65f){
                hit = false;
                SetReward(-50f);    // 도달 실패하면 -
                EndEpisode();
                statusText.text = "목표점 도달 실패";
            }
            
            else{
                float reward = preDist - distance;
                AddReward(reward);
                preDist = distance;  
                rewardText.text = "보상: "+reward;
            }
        
        yText.text = "y축 차이: "+Mathf.Abs(goalTrans.position.y - agentTrans.position.y);
        xText.text = "x축 차이: "+Mathf.Abs(goalTrans.position.x - agentTrans.position.x);
        zText.text = "z축 차이: "+Mathf.Abs(goalTrans.position.z - agentTrans.position.z);
        goalNum.text = "목표점의 번호: "+ goal;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(agentTrans.position - goalTrans.position);
        sensor.AddObservation(DroneAgent_Rigidbody.velocity);
        sensor.AddObservation(DroneAgent_Rigidbody.angularVelocity);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
        continuousActionsOut[2] = Input.GetAxis("Mouse ScrollWheel");
    }

    float DecisionWaitingTime = 0.01f;
    float m_currentTime = 0f;

    public void WaitTimeInference(int action)
    {
        if(Academy.Instance.IsCommunicatorOn)
        {
            RequestDecision();
        }
        else
        {
            if(m_currentTime >= DecisionWaitingTime)
            {
                m_currentTime = 0f;
                RequestDecision();
            }
            else
            {
                m_currentTime += Time.fixedDeltaTime;
            }
        }
    }
}