/*
 *  CST_Path.cpp
 *  CST_G1
 *
 *  Created by Chris Tharpe on 8/29/11.
 *  Copyright 2011 Christopher S. Tharpe. All rights reserved.
 *
 */

#include "CST_Path.h"

CST_Path::CST_Path()
{
	Init();
};

CST_Path::~CST_Path()
{
    //Vertices no longer allocated directly for path, they are stored in m_cSlidingArray
    //set m_pVertices to zero to prevent delete[] call:
    m_pVertices = 0;
};

void CST_Path::Init()
{
	//m_fDeltaT is not delta time, it's delta of parametric variable t:
	//should be small enough so that (F(t + deltaT) - F(t))/deltaT ~= f'(t)
    //but not so small that F(t + deltaT) - F(t) ~= 0.0 (if that's the
    //case, you get no movement)
    
    if(m_pFrustum)
    {
        CST_Rect sR;
        m_pFrustum->GetFrustumBoundaryAtAltitude(&sR, -(m_pFrustum->far));
        GLfloat d = fabs(sR.p1[0] - sR.p2[0]);
        if(m_nMinorDisplayDim > 0.0 && d > 0.0)
           // m_fdeltaT = d / ((GLfloat)m_nMinorDisplayDim * 1000.0);
            m_fdeltaT = d / ((GLfloat)m_nMajorDisplayDim * 500.0);
        else 
        {
            m_fdeltaT = 0.02;
        }

    }
    else 
    {
        m_fdeltaT = 0.02;
    }
    
 
    m_fdTSign = 1.0;  //this controls direction along path, e.g. clockwise or counter-clockwise
	
	//just a test radius
	m_fRadius = 1000.0;
    m_bDrawPath = false;
    m_eDrawMode = GL_LINES;
    //m_eDrawMode = GL_LINE_LOOP;
    m_bFirstTime = true;
    m_nMaxPointsInDrawnPath = CST_MAX_POINTS;
	
	ClearTerms();
	
	//Default for circle
	SetA(3, m_fRadius);
	SetB(2, m_fRadius/2.0);
	SetC(1, 40.0);
	//GLfloat test = 0.0;
	//test = m_pFrustum->near;
	SetRepeat(true);
	SetUpperLimit(2.0 * CST_PI);
	SetLowerLimit(0.0);
    
    m_bRotatePath = false;
    
    InitializePoint(m_pPathCentroid);
	
};

void CST_Path::SetMaxPointsForDrawnPath(int max /*=0*/)
{
    if(max < 1)
        max = 1;
    if(max > CST_MAX_POINTS)
        max = CST_MAX_POINTS;
    
    m_nMaxPointsInDrawnPath = max;
};

void CST_Path::CreateVertices()
{
    if(m_bDrawPath)
    {
        if(!m_cSlidingArray.IsArrayCreated())
        {
            //allocate memory for point and index arrays
            m_bDisplay = true;
            m_pVertices = 0;
            m_cSlidingArray.CreateArray(m_nMaxPointsInDrawnPath, CST_DIMENSION);
            m_pIndices = new GLushort[2 * (m_nMaxPointsInDrawnPath - 1)];
            
            m_nVertexArraySize = 0;
            m_nPoints = 0;
            m_nIndexArraySize = 0;

            for(int i = 0; i < 2 * (m_nMaxPointsInDrawnPath - 1); i++)
                *(m_pIndices + i) = 0;
        }
    }
};

void CST_Path::ClearTerms(void)
{
	//clears coefficients of parametric function
	for(int i = 0; i < (CST_NUMBER_OF_TERMS /*- 1*/); i++)
	{
		A[i] = B[i] = C[i] = 0.0;
	}
    //Prevent division by zero in FX(t), FY(t), FZ(t) - see below
    if(CST_NUMBER_OF_TERMS > 6)
    {
        A[6] = B[6] = C[6] = 1.0;
    }
};

void CST_Path::SetMFDeltaTSign(void)
{
    //This sets the sign of m_fDeltaT & dT
    //should be called AFTER SetUpperLimit() and
    //SetLowerLimit() are called for this path
    //(see CST_Path.h)
    
    //For example, if path is a circle, and if UpperLimit = 0 and LowerLimit = 2 PI, then
    //you get clockwise motion
    //BUT, if UpperLimit = 2 PI and LowerLimit = 0, then you get counter-clockwise motion
    
    m_fdTSign = 1.0;
    
    if(m_fLowerLimit > m_fUpperLimit)
        m_fdTSign = -1.0;
    
};

bool CST_Path::SetA(int index /*=0*/, GLfloat value /*=0.0*/)
{
	//returns true if value set, false otherwise
	bool result = false;
	
	if(index >= 0 && index < CST_NUMBER_OF_TERMS)
	{
		A[index] = value;
		result = true;
	};
	
	return result;
};

bool CST_Path::SetB(int index /*=0*/, GLfloat value /*=0.0*/)
{
	//returns true if value set, false otherwise
	bool result = false;
	
	if(index >= 0 && index < CST_NUMBER_OF_TERMS)
	{
		B[index] = value;
		result = true;
	};
	
	return result;
};

bool CST_Path::SetC(int index /*=0*/, GLfloat value /*=0.0*/)
{
	//returns true if value set, false otherwise
	bool result = false;
	
	if(index >= 0 && index < CST_NUMBER_OF_TERMS)
	{
		C[index] = value;
		result = true;
	};
	
	return result;
};

GLfloat CST_Path::GetA(int index /*=0*/)
{
    GLfloat result = 0.0;
    
    if(index >= 0 && index < CST_NUMBER_OF_TERMS)
    {
        result = A[index];
    }
    
    return result;
};

GLfloat CST_Path::GetB(int index /*=0*/)
{
    GLfloat result = 0.0;
    
    if(index >= 0 && index < CST_NUMBER_OF_TERMS)
    {
        result = B[index];
    }
    
    return result;
};

GLfloat CST_Path::GetC(int index /*=0*/)
{
    GLfloat result = 0.0;
    
    if(index >= 0 && index < CST_NUMBER_OF_TERMS)
    {
        result = C[index];
    }
    
    return result;
};

//NOTE:

//CAREFUL with FOLLOWING, FOREGOING DIVISION BY ZERO CHECK TO INCREASE
//SPEED: (have default for A[6] = B[6] = C[6] = 1.0.

GLfloat CST_Path::FX(GLfloat t)
{
	return A[0] + A[1] * t + A[2] * sin(t) + A[3] * cos(t)  
          + A[4] * (A[5] - (1.0 / (1.0 + exp(-t / A[6]))));  
};

GLfloat CST_Path::FY(GLfloat t)
{
	return B[0] + B[1] * t + B[2] * sin(t) + B[3] * cos(t)  
     + B[4] * (B[5] - (1.0 / (1.0 + exp(-t / B[6]))));
};

GLfloat CST_Path::FZ(GLfloat t)
{
    //C[7] used to test tilting about Y axis
    
	return C[0] + C[1] * t + C[2] * sin(t) + C[3] * cos(t) 
    + C[4] * (C[5] - (1.0 / (1.0 + exp(-t / C[6])))) + C[7] * FX(t);
};

void CST_Path::FParametric(GLfloat t /*= 0*/, GLfloat *pResult /*= 0*/)
{
	//Method assumes 3-dimensional points, and that memory for pResult is allocated/freed elsewhere (or on stack)
	if(pResult)
	{
		*pResult = FX(t);
		*(pResult + 1) = FY(t);
		*(pResult + 2) = FZ(t);
        
        if(m_bRotatePath)
            ApplyTransformClientFast(pResult, 1);
	}
};

void CST_Path::UpdatePosition(CST_Motion *pMotion /*=0*/, CST_Motion *pMotionAbsolute /*=0*/,GLfloat *pDirectionVector /*=0*/)
{
	if(pMotion)
	{
		pMotion->m_fVx = 0.0;
		pMotion->m_fVy = 0.0;
		pMotion->m_fVz = 0.0;
		 
		if(LimitParameter(pMotion))
		{
			GLfloat P0[CST_DIMENSION];
			GLfloat P1[CST_DIMENSION];
			GLfloat P2[CST_DIMENSION];
			GLfloat magnitude = 0.0;
			GLfloat distance = 0.0;
			GLfloat dT = 0.0;
			
			InitializePoint(P0);
            InitializePoint(P1);
            InitializePoint(P2);
            
			distance = pMotion->m_fSpeed * pMotion->m_fdeltaTime;
            
            GLfloat min_distance = CST_MIN_DISTANCE;
            
            min_distance = fabs(pMotion->m_fSpeed) / 68.0;
			
			if(distance != 0)
			{
				FParametric(pMotion->m_ft, P0);
				FParametric(pMotion->m_ft + m_fdTSign * m_fdeltaT, P1);
                
				SubtractPoints(P0, P1, P1);
				//At this point, P1 should be the tangent vector
				//and m_fdeltaT should be small enough so that
				//the secant vector ~= tangent vector
				
				//dividing P1 by m_fdeltaT should then give you an
				//approximation of f'(t).
				DivideVectorByScalar(P1, m_fdTSign * m_fdeltaT);
				
				//P1 is now an approximation of f'(t)
				
				//distance = dT * |f'(t)|
				//dT = distance / |f'(t)|
				
				magnitude = Magnitude(P1);
				if(magnitude)
					dT = m_fdTSign * (distance / magnitude);
                
				pMotion->m_ft += dT;
                
				LimitParameter(pMotion);

				FParametric(pMotion->m_ft, P2);
				//This gives the actual final destination of the object
				//Transform function is a velocity, though, so I actually need 
				//to divide P0 by pMotion->m_fdeltaTime
                
                //set up vertices for drawing, if m_bDrawPath == true
                if(m_bDrawPath == true)
                {
                    GLfloat draw_point[CST_DIMENSION];
                    draw_point[0] = draw_point[1] = draw_point[2] = 0.0;
                
                    if(m_bFirstTime == true)
                    {
                        m_bFirstTime = false;
                        draw_point[0] = *P0;
                        draw_point[1] = *(P0 + 1);
                        //draw_point[2] = pMotionAbsolute->m_fVz;
                        draw_point[2] = *(P0 + 2);
                        AddPointToBeDrawn(draw_point);
                    }
 
                    if(distance > min_distance)
                    {
                        draw_point[0] = *P2;
                        draw_point[1] = *(P2 + 1);
                        //draw_point[2] = pMotionAbsolute->m_fVz;
                        draw_point[2] = *(P0 + 2);
                        AddPointToBeDrawn(draw_point);
                    }
                }
				
				if(pDirectionVector && pMotionAbsolute)
				{
					//if pDirectionVector is fed in, assume that game object is to be rotated
					//to align with Path, in addition to just being moved along it.
                    //pDirectionVector is vector that will be aligned with path
  
					GLfloat localpoint[3];
                    GLfloat P3[CST_DIMENSION];
                    InitializePoint(P3);
                    InitializePoint(localpoint);
                    
                    //P3 is just slightly further along path than P2
                    FParametric(pMotion->m_ft + m_fdTSign * m_fdeltaT, P3);
                    
                    AddPoints(P2, pDirectionVector, localpoint);
            

                    CrossProductAndAngleNormalized(P2, localpoint, P2, P3, pMotionAbsolute);
                    
					pMotionAbsolute->m_fAngle *= (180.0 / CST_PI);

					//NOTE: will still have to call rotation translation in calling method
				}
				
				SubtractPoints(P0, P2, P2);
				
				if(pMotion->m_fdeltaTime)
					DivideVectorByScalar(P2, pMotion->m_fdeltaTime);
				
				pMotion->m_fVx = P2[0];
				pMotion->m_fVy = P2[1];
				pMotion->m_fVz = P2[2];
			}
		}
	}
};

bool CST_Path::LimitParameter(CST_Motion *pMotion /*=0*/)
{
	//Keeps parametric variable from getting out of range
	//This method will need to be specialized for each type of CST_Path	
    
    //returns false when limits reached
	
	bool result = true;
	
	if(pMotion)
	{
		if(m_bRepeat)
		{
            if(m_fdTSign < 0.0)
            {
                //if sign is negative, upper and lower limits are switched
                if(pMotion->m_ft > m_fLowerLimit)
                {
                    pMotion->m_ft = m_fUpperLimit;
                    ResetSlidingArray();
                    pMotion->m_bPathLimitReached = true;
                }
                
                if(pMotion->m_ft < m_fUpperLimit)
                {
                    pMotion->m_ft = m_fLowerLimit;
                    ResetSlidingArray();
                     pMotion->m_bPathLimitReached = true;
                }
            }
            else
            {
                if(pMotion->m_ft > m_fUpperLimit)
                {
                    pMotion->m_ft = m_fLowerLimit;
                    ResetSlidingArray();
                     pMotion->m_bPathLimitReached = true;
                }
                
                if(pMotion->m_ft < m_fLowerLimit)
                {
                    pMotion->m_ft = m_fUpperLimit;
                    ResetSlidingArray();
                     pMotion->m_bPathLimitReached = true;
                }
			}	
		}
		else
		{
			if(pMotion->m_ft < m_fLowerLimit || pMotion->m_ft > m_fUpperLimit)
			{
				result = false;
                pMotion->m_bPathLimitReached = true;
                
                if(m_fdTSign > 0.0)
                    pMotion->m_ft = m_fUpperLimit;
                else 
                    pMotion->m_ft = m_fLowerLimit;
                
                ResetSlidingArray();
			}
		}
	}
	
	return result;
};

void CST_Path::GetStartPoint(GLfloat *pResult /*=0*/)
{
    //Method should only be called after m_fLowerLimit and m_fUpperLimit are set
  if(pResult)
      FParametric(m_fLowerLimit, pResult);
};

void CST_Path::GetEndPoint(GLfloat *pResult /*=0*/)
{
    //Method should only be called after m_fLowerLimit and m_fUpperLimit are set    
    if(pResult)
        FParametric(m_fUpperLimit, pResult);    
};

void CST_Path::AddPointToBeDrawn(GLfloat *pPoint /*=0*/)
{
    CreateVertices();
    
    if(pPoint && m_pIndices && m_bDrawPath)
    {
        AddPointToSlidingArray(pPoint);
        m_pVertices = m_cSlidingArray.GetPointerToRead();
        m_nVertexArraySize = m_cSlidingArray.GetTotalSizeOfActiveItems();
        m_nPoints = m_cSlidingArray.GetCurrentlyActiveItems();
        
        if(m_nPoints > 1 && m_nPoints < m_nMaxPointsInDrawnPath)
        {
            *(m_pIndices + m_nIndexArraySize) = m_nPoints - 2;
            *(m_pIndices + m_nIndexArraySize + 1) = m_nPoints - 1;
            m_nIndexArraySize += 2;
        }    
    }    
};

void CST_Path::AddPointToSlidingArray(GLfloat *pPoint /*=0*/)
{
    if(pPoint)
    {
        m_cSlidingArray.AddItem(pPoint);
    }
};

void CST_Path::ResetSlidingArray()
{
    if(m_cSlidingArray.IsArrayCreated())
    {
        m_cSlidingArray.Reset();
        m_nIndexArraySize = 0;  //reseting this objects m_nIndexArraySize - used to 
                                //draw verts in sliding array
    }
};

void CST_Path::SetPathCentroid(GLfloat* pPoint /*=0*/)
{
    if(pPoint)
    {
        CopyPoint(m_pPathCentroid, pPoint);
    }
};

GLfloat* CST_Path::GetPointerToPathCentroid()
{
    return m_pPathCentroid;
};

void CST_Path::SetupPathRotation(GLfloat *pCentroid /*=0*/, CST_Motion *pMotion /*=0*/)
{
    //used to setup m_pTransform which will rotate path
    //rotation will take place about centroid point
    
    //method assumes that since you're calling it, that m_bRotatePath will be true
    
    if(pCentroid && pMotion)
    {
        m_bRotatePath = true;
        
        InitializeTransformMatrix(m_pTransform);
        CreateTransform2();
        CreateTransform3();
        
        SetPathCentroid(pCentroid);

        CreateTransformForRotationAboutPoint(pCentroid, pMotion, m_pTransform); 
    }
};

void CST_Path::SetParameterLimitOffScreen(GLfloat *pParameter /*= 0*/, GLfloat sign /*= 1.0*/, GLfloat safety_distance /*= 0.0*/)
{
    //Method assumes that m_pPathCentroid is set
    
    //CAUTION: must be careful with this - if path is circular/closed and fully inside screen
    //this method could result in INFINITE LOOP
    
    if(m_pFrustum  && pParameter)
    {
        GLfloat percent_screen_size = 0.05;  //percent screen size for determining parameter variable increment size
        GLfloat t = 0.0;
        GLfloat t_inc = 0.0;        
        
        GLfloat distance = 0.0;
        
        distance = m_pFrustum->GetDistanceCorrespondingToScreenPercentAtZ(*(m_pPathCentroid + 2), percent_screen_size);
        
        CST_Rect rect;
        m_pFrustum->GetFrustumBoundaryAtAltitude(&rect, *(m_pPathCentroid + 2));
        GLfloat diagonal_distance = rect.DiagonalDistance();
        
        int iteration_limit = (int) (diagonal_distance / distance);
        iteration_limit *= 3;
        
        GLfloat point_1[CST_DIMENSION];
        GLfloat point_2[CST_DIMENSION];
        
        InitializePoint(point_1);
        InitializePoint(point_2);
        
        FParametric(t, point_1);
        FParametric(t + m_fdeltaT, point_2);
        
        GLfloat distance_between_points = 0.0;
        
        distance_between_points = DistanceBetweenPoints(point_1, point_2);
        
        if(distance_between_points > 0.0)
        {
            t_inc = sign * (m_fdeltaT / distance_between_points) * distance;
            
            int iteration_count = 0;
            
            while(m_pFrustum->IsPointInFrustum(point_1) && iteration_count < iteration_limit)
            {
                t += t_inc;
                FParametric(t, point_1);
                
                iteration_count++;
            }
            
            if(safety_distance != 0.0)
            {
                FParametric(t + m_fdeltaT, point_2);
                
                distance_between_points = DistanceBetweenPoints(point_1, point_2);
                
                if(distance_between_points > 0.0)
                {
                    t_inc = sign * (m_fdeltaT / distance_between_points) * safety_distance;
                    
                    t += t_inc;
                }
            }
            
            *pParameter = t;
        }
    }
};

void CST_Path::SetUpperLimitOffScreen(GLfloat safety_distance /*=0.0*/)
{
    GLfloat sign = 1.0;
    
    sign = m_fdTSign;
    
    SetParameterLimitOffScreen(&m_fUpperLimit, sign, safety_distance);
};

void CST_Path::SetLowerLimitOffScreen(GLfloat safety_distance /*=0.0*/)
{
    GLfloat sign = -1.0;
    
    sign = -1.0 * m_fdTSign;
    
    SetParameterLimitOffScreen(&m_fLowerLimit, sign, safety_distance);    
};

void CST_Path::SetUpperAndLowerLimitOffScreen(GLfloat safety_distance /*=0.0*/)
{
    SetLowerLimitOffScreen(safety_distance);
    SetUpperLimitOffScreen(safety_distance);
};
