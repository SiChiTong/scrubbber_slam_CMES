
//�������
attribute vec4  vPosition; 
attribute vec4  vColor;


uniform mat4  matModel;
uniform mat4  matProject;

//�������
varying vec4  varColor;
varying float varHigh; 

void main()                                
{               
		varColor = vColor;
		varHigh = vPosition.z;
 	 	gl_Position = matProject * matModel* vPosition;
  	    gl_PointSize= 1.0;
}                                          
