using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/**
 * beta 0.0.11
 * wrirtten by yasu80
 * 20170723
 * 
 * dilatant force_ver1 added
 * __Under__developping__
 * 
 * .11_fixed input vector length 
 */

public class Sticky : MonoBehaviour
{
    public GameObject me;
    public GameObject you;
    Vector3 base_me = new Vector3(0, 0, 0);
    Vector3 base_you = new Vector3(0, 0, 0);
    Vector3 stuckked_me = new Vector3(0, 0, 0);
    Vector3 stuckked_you = new Vector3(0, 0, 0);
    Vector3 base_collision = new Vector3(0, 0, 0);
    //Vector3 relative_vector = new Vector3  (0,0,0);
    Vector3 relative_vector;
    Vector3 myVelo = new Vector3(0, 0, 0);
    Vector3 yourVelo = new Vector3(0, 0, 0);
    Vector3 v;

    float gain = 0;

    int k = 1000;

    public Rigidbody myRigidBody;
    public Rigidbody yourRigidBody;

    void OnCollisionEnter(Collision collision)
    {
        ContactPoint contact = collision.contacts[0];//Easy_Plus
        Quaternion rotate_me = Quaternion.FromToRotation(Vector3.up, contact.point);//Poop
        stuckked_you = contact.point;
        base_collision = stuckked_you;
        you = collision.gameObject;//Get Collision Object
        yourRigidBody = you.GetComponent<Rigidbody>();
    }

    // Use this for initialization
    void Start()
    {
        myRigidBody = GetComponent<Rigidbody>();
        //me = this.GetComponent<GameObject>();
    }

    // Update is called once per frame
    /*void Update () {
        //relative_vector = you.transform.position - me.GetComponent<Transform.osition>();
        a = you.GetComponent<Transform>();
        b = me.GetComponent<Transform>();
        v = a.transform - b.transform;//NO GOOD
        myRigidBody.AddForce(relative_vector.normalized * 100);
        Debug.Log(relative_vector.x);
        Debug.Log(relative_vector.y);
        Debug.Log(relative_vector.z);
        Debug.Log("(^m^)");
    }*/
    void FixedUpdate()
    {
        relative_vector = you.transform.position - me.transform.position;
        float distance = relative_vector.magnitude;
        //float gravity = k / distance;
        myVelo = myRigidBody.velocity;
        yourVelo = yourRigidBody.velocity;
        gain = (float)fruidForce(relative_vector.magnitude, k, relative_vector, 100.0,100.0, yourVelo - myVelo,  100.0, 100.0);
        myRigidBody.AddForce(gain * (relative_vector.normalized), ForceMode.Force);
    }
    
    double fruidForce(double dist, double sigma, Vector3 velo_relative, double length_bond, double length_friction, Vector3 dist_vector,//wantfix
        double k_bond, double k_friction)
    {
        double force = 0;
        force = k_friction * ( 1 / Mathf.Sqrt(2*Mathf.PI*Mathf.Pow((float)length_friction, 2.0f)) * Mathf.Exp( -(Mathf.Pow((float)dist,2.0f)) / (2 * Mathf.Pow((float)length_friction, 2.0f)) )*   Vector3.Dot(velo_relative, dist_vector)
                +  k_bond * (1 / Mathf.Sqrt(2 * Mathf.PI * Mathf.Pow((float)length_bond, 2.0f)) * Mathf.Exp(-(Mathf.Pow((float)dist, 2.0f)) / (2 * Mathf.Pow((float)length_bond, 2.0f))) * dist ) );
        Debug.Log("Force:");
        Debug.Log(force);
        return force;
    }
    
    bool IsBreak(float limit, float force)
    {
        if (limit > force) return true;
        else return false;
    }

}