#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace std;

#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main( )
{
    USING_NAMESPACE_ACADO

    // Variables:
	DifferentialState   x     ;  // position x
	DifferentialState   y     ;  // position y
	DifferentialState   z     ;  // position z
	DifferentialState   vx    ;  // velocity x 
    DifferentialState   vy    ;  // velocity y
    DifferentialState   z1    ;  // velocity z state z1
    DifferentialState   z2    ;  // velocity z state sz2
    DifferentialState   psi   ;  // angle roll
    DifferentialState   theta ;  // angle pitch
	Control             psid  ;  // command roll
    Control             thetad;  // command theta
    Control             vzd   ;  // command vertical velocity

    const double     g = 9.81;  // the gravitational constant 
	const double     m = 0.50;  // the mass of drone (kg)
    const double     pi = 3.14; // pi

    // identification result
    const double Ax =   0.1840; // air drag
    const double Ay =   0.2007; // air drag
    const double va1 =  -2.292; // vertical velocity model
    const double va2 =  -2.550; // vertical velocity model
    const double va3 =   9.742; // vertical velocity model
    const double va4 =  -5.845; // vertical velocity model
    const double vb1 = 0.00642; // vertical velocity model
    const double vb2 = -0.6570; // vertical velocity model
    const double vc1 =  17.650; // vertical velocity model
    const double vc2 = -0.3225; // vertical velocity model
    const double vd =        0; // vertical velocity model
    const double ra =  -6.7529; // roll model
    const double rb =  6.86585; // roll model
    const double pa =  -6.7317; // pitch model
    const double pb =  6.85417; // pitch model

    // Model equations:
	DifferentialEquation f; 

	f << dot(  x  ) == vx;
	f << dot(  y  ) == vy;
    f << dot(  z  ) == vc1 * z1 + vc2 * z2 + vd * vzd;
	f << dot(  vx ) == - Ax * vx + m * g * theta;
    f << dot(  vy ) == - Ay * vy + m * g * psi;
	f << dot(  z1 ) == va1 * z1 + va2 * z2 + vb1 * vzd;
    f << dot(  z2 ) == va3 * z1 + va4 * z2 + vb2 * vzd;
	f << dot( psi ) == ra * psi + rb * psid;
    f << dot(theta) == pa * theta + pb * thetad;

    // Reference functions and weighting matrices:
	Function h, hN;
	h << x << y << z << psid << thetad << vzd;
	hN << x << y << z;

    // Reference
    DVector r(3); 
    r.setAll( 0.0 );

	DMatrix W = eye<double>( h.getDim());
	W(3,3) = 0.01;
	W(4,4) = 0.01;
	W(5,5) = 0.01;
	DMatrix WN = eye<double>( hN.getDim() );

	//
	// Optimal Control Problem
	//
	const double TEnd = 2.0;
	const double Ts = 1 / 15.0;

	OCP ocp( 0.0,  TEnd , TEnd / Ts);

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( f );

	ocp.subjectTo( -1 <= vx <= 1 ); 				// m/s
	ocp.subjectTo( -1 <= vy <= 1 ); 				// m/s
	ocp.subjectTo( -pi / 36 <= psid <= pi / 36 ); 	// deg
	ocp.subjectTo( -pi / 36 <= thetad <= pi / 36 ); // deg
	ocp.subjectTo( -1.0 <= vzd <= 1.0 ); 			// m/s

	// Export the code:
	OCPexport mpc( ocp );
	
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); 
    mpc.set( INTEGRATOR_TYPE, INT_IRK_GL2);
    mpc.set( QP_SOLVER, QP_QPOASES);
    mpc.set( HOTSTART_QP, YES);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);
    mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);

	if (mpc.exportCode( "mpc_flight_controller_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
