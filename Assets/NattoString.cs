using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NattoString : MonoBehaviour {    
    public double lo;
    public double p;
    public double ganma;
    public double k;
    
    public double variance_combining(double lo, double p, double ganma, double k, double x)
    {
        double retval = -1;

        if (lo < 0 || p <= 0 || ganma < 0 || k < 0 || x < 0) return retval;
        retval = p * Mathf.Exp((-Mathf.Pow((float)(x - lo), 2) / (2 * Mathf.Pow((float)ganma, 2))));        
        return retval;
    }


    double variance_friction(double lo, double p, double ganma, double k, double x, double x_dot)
    {
        double retval = -1;

        if (lo < 0 || p <= 0 || ganma < 0 || k < 0 || x < 0) return retval;


        return retval;
    }

    // Update is called once per frame
    void Update () {
		
	}

}
