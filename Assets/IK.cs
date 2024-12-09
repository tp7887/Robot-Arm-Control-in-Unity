using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IK : MonoBehaviour
{
    private GameObject cube;

    public float Px;
    public float Py;
    public float Pz;
    public float speed = 5f;

    public ArticulationBody joint1;
    public ArticulationBody joint2;
    public ArticulationBody joint3;

    private float Px_r1;
    private float x0;
    private float y0;
    private float z0;

    private float Phi1;
    private float Phi1_0;
    private float target_Phi1;

    private float CosPhi3;
    private float SinPhi3;
    private float Phi3;
    private float Phi3_0;
    private float target_Phi3;

    private float Phi2;
    private float Phi2_0;
    private float target_Phi2;

    private ArticulationBody[] articulationChain;
    private float l1;
    private float l2;

    // Start is called before the first frame update
    void Start()
    {
        cube = GameObject.Find("Cube");
        Px = cube.transform.position.x;
        Py = cube.transform.position.y;
        Pz = cube.transform.position.z;
        x0 = GameObject.Find("link2").transform.position.x;
        y0 = GameObject.Find("link2").transform.position.y;
        z0 = GameObject.Find("link2").transform.position.z;

        l1 = Vector3.Distance(joint2.transform.position, joint3.transform.position);
        l2 = Vector3.Distance(joint3.transform.position, cube.transform.position);

        // Initial setup
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        foreach (ArticulationBody body in articulationChain)
        {
            body.jointFriction = 10;
            body.angularDamping = 10;
            ArticulationDrive body_drive = body.xDrive;
            body_drive.forceLimit = 100000000000;
            body_drive.damping = 10000;
            body_drive.stiffness = 100000;
            body.xDrive = body_drive;
        }

        // Initial configuration of the robot
        Phi1 = Phi1_0 = (Mathf.Atan2((Pz - z0), (Px - x0))) * 180 / Mathf.PI;

        Px_r1 = Px * Mathf.Cos(Phi1 * Mathf.PI / 180) + Py * Mathf.Sin(Phi1 * Mathf.PI / 180);

        CosPhi3 = (Mathf.Pow((Px_r1 - x0), 2) + Mathf.Pow((Py - y0), 2) - Mathf.Pow(l1, 2) - Mathf.Pow(l2, 2)) / (2 * l2 * l1);
        SinPhi3 = -Mathf.Sqrt(1 - Mathf.Pow(CosPhi3, 2));
        Phi3 = Phi3_0 = (Mathf.Atan2(SinPhi3, CosPhi3)) * 180 / Mathf.PI;

        Phi2 = Phi2_0 = (Mathf.Atan2((Py - y0), (Px_r1 - x0)) + Mathf.Atan2(-l2 * SinPhi3, l1 + l2 * CosPhi3)) * 180 / Mathf.PI;
    }

    // Update is called once per frame
    void Update()
    {
        // Inverse kinematics
        if (Input.GetKeyDown(KeyCode.Return))
        {
            Phi1 = (Mathf.Atan2((Pz - z0), (Px - x0))) * 180 / Mathf.PI;

            Px_r1 = Px * Mathf.Cos(Phi1 * Mathf.PI / 180) + Pz * Mathf.Sin(Phi1 * Mathf.PI / 180);

            if ((Px * Px + Pz * Pz < Mathf.Pow(l1 + l2, 2)))
            {
                CosPhi3 = (Mathf.Pow((Px_r1 - x0), 2) + Mathf.Pow((Py - y0), 2) - Mathf.Pow(l1, 2) - Mathf.Pow(l2, 2)) / (2 * l2 * l1);
                SinPhi3 = -Mathf.Sqrt(1 - Mathf.Pow(CosPhi3, 2));
                Phi3 = (Mathf.Atan2(SinPhi3, CosPhi3)) * 180 / Mathf.PI;

                Phi2 = (Mathf.Atan2((Py - y0), (Px_r1 - x0)) + Mathf.Atan2(-l2 * SinPhi3, l1 + l2 * CosPhi3)) * 180 / Mathf.PI;
                target_Phi1 = Phi1 - Phi1_0;
                target_Phi2 = -Phi2 + Phi2_0;
                target_Phi3 = Phi3 - Phi3_0;
                //Debug.Log(target_Phi3);
            }
        }

        joint_drive(joint1, target_Phi1);
        joint_drive(joint2, target_Phi2);
        joint_drive(joint3, target_Phi3);
        
        //Debug.Log(cube.transform.position);
    }

    private void joint_drive(ArticulationBody joint, float target)
    {
        int direction = 0;
        if (joint.jointType != ArticulationJointType.FixedJoint)
        {
            ArticulationDrive currentDrive = joint.xDrive;
            if (currentDrive.target < target)
            {
                direction = 1;
            }
            else if (currentDrive.target > target)
            {
                direction = -1;
            }
            else if (Mathf.Abs(currentDrive.target - target) < 0.1f)
            {
                direction = 0;
            }

            float newTargetDelta = (int)direction * Time.fixedDeltaTime * speed;

            if (joint.jointType == ArticulationJointType.RevoluteJoint)
            {
                if (joint.twistLock == ArticulationDofLock.LimitedMotion)
                {
                    if (newTargetDelta + currentDrive.target > currentDrive.upperLimit)
                    {
                        currentDrive.target = currentDrive.upperLimit;
                    }
                    else if (newTargetDelta + currentDrive.target < currentDrive.lowerLimit)
                    {
                        currentDrive.target = currentDrive.lowerLimit;
                    }
                    else
                    {
                        currentDrive.target += newTargetDelta;
                    }
                }
                else
                {
                    currentDrive.target += newTargetDelta;
                }
            }
            joint.xDrive = currentDrive;
        }
    }
    public void OnGUI()
    {
        GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
        centeredStyle.alignment = TextAnchor.UpperCenter;

        centeredStyle.fontSize = 20;
        centeredStyle.normal.textColor = Color.black;

        GUI.Label(new Rect(Screen.width / 2 - 200, 0, 400, 100), "End effector coordinate: " + cube.transform.position + ".", centeredStyle);        
    }
}
