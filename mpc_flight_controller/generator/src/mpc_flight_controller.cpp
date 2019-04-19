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

    const float     g = 9.81;  // the gravitational constant 
	const float     m = 0.50;  // the mass of drone (kg)
    const float     pi = 3.14; // pi

    // identification result
    const float Ax =   0.1840; // air drag
    const float Ay =   0.2007; // air drag
    const float va1 =  -2.292; // vertical velocity model
    const float va2 =  -2.550; // vertical velocity model
    const float va3 =   9.742; // vertical velocity model
    const float va4 =  -5.845; // vertical velocity model
    const float vb1 = 0.00642; // vertical velocity model
    const float vb2 = -0.6570; // vertical velocity model
    const float vc1 =  17.650; // vertical velocity model
    const float vc2 = -0.3225; // vertical velocity model
    const float vd =        0; // vertical velocity model
    const float ra =  -6.7529; // roll model
    const float rb =  6.86585; // roll model
    const float pa =  -6.7317; // pitch model
    const float pb =  6.85417; // pitch model

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
	h << psid << thetad << vzd;
	hN << x << y << z;

    // Reference
    DVector r(3); 
    r.setAll( 0.0 );

	DMatrix W = eye<double>(h.getDim());
	DMatrix WN = eye<double>(hN.getDim()) * 10;

	//
	// Optimal Control Problem
	//
	const float TEnd = 1.5;
	const float Ts = 1 / 15.0;

	OCP ocp( 0.0,  TEnd , TEnd / Ts);

	ocp.minimizeLSQ(W, h) * 0.01;
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( f );

	ocp.subjectTo( -0.25 <= vx <= 0.25 ); 				// m/s
	ocp.subjectTo( -0.25 <= vy <= 0.25 ); 				// m/s
	ocp.subjectTo( -pi / 36 <= psid <= pi / 36 ); 	// deg
	ocp.subjectTo( -pi / 36 <= thetad <= pi / 36 ); // deg
    ocp.subjectTo( -0.5 <= vzd <= 0.5 ); // deg


	// Export the code:
	OCPexport mpc( ocp );
	
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); 
    mpc.set( INTEGRATOR_TYPE, INT_IRK_GL2);
    mpc.set( QP_SOLVER, QP_QPOASES);
    mpc.set( HOTSTART_QP, NO);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);
    mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
    // mpc.set( CG_USE_OPENMP, NO);
    // mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    // mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

	if (mpc.exportCode( "mpc_flight_controller_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
