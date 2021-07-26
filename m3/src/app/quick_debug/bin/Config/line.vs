
//输入变量
attribute vec4 vPosition; 
attribute vec3 vNomral;
attribute vec3 vColor;       

uniform mat4  matModel;
uniform mat4  matProject;
uniform vec3  vecParam;

//输出变量
varying vec3   varColor;
varying float  varLineWidth;
varying float  varDistance;

void main()                                
{	 
	varColor = vColor;

	varLineWidth = vecParam.x;
	varDistance =  vNomral.z;

	vec4 delta = vec4(vNomral.xy * varLineWidth, 0, 0);
	
	mat4 mvpMat =  matProject * matModel;

	gl_Position = mvpMat*(vPosition + delta);

	vec4 v =  mvpMat[0]* vecParam.x;

	varLineWidth = length(v);
}                                          
