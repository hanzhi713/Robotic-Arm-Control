# Mechanism

To reach point D in space, the robotic arm first needs to rotate <img src="http://latex.codecogs.com/svg.latex?\inline&space;\large&space;\varphi" title="\large \varphi" /> degrees so that the linkages of the arm and the target point are coplanar (i.e. they all line on the blue plane). 

<img src="http://latex.codecogs.com/svg.latex?\large&space;\varphi&space;=&space;\arctan{\frac{y}{x}}" title="\large \varphi = \arctan{\frac{y}{x}}" />

<img src="demo/three-to-two.png" width="600px">

The spatial coordinate of the target point D is mapped to a coordinate on the blue plane, which could be calculated as follow

<img src="http://latex.codecogs.com/svg.latex?\large&space;f:&space;\mathbb{R}^3&space;\rightarrow&space;\mathbb{R}^2&space;\\&space;f(x,&space;y,&space;z)&space;=&space;(\sqrt{x^2&space;&plus;&space;y^2},\&space;z)" title="\large f: \mathbb{R}^3 \rightarrow \mathbb{R}^2 \\ f(x, y, z) = (\sqrt{x^2 + y^2},\ z)" />

Consequently, this problem is simplied to a two dimensional route planning problem. 

<img src="demo/2d.svg" width="500px">

As shown in the diagram above, the solution for this problem is not unique. A particular position of the first segment (l<sub>1</sub>) corresponds to a particular solution. However, not all positions of point B are suitable. We require:

<img src="http://latex.codecogs.com/svg.latex?\inline&space;\large&space;\left\{\begin{matrix}&space;l_2&space;&plus;&space;l_3&space;>&space;BD&space;\\&space;\sqrt{l_2^2&space;&plus;&space;l_3^2}&space;<&space;BD&space;\end{matrix}\right." title="\large \left\{\begin{matrix} l_2 + l_3 > BD \\ \sqrt{l_2^2 + l_3^2} < BD \end{matrix}\right." />

The first condition ensures that point B won't be so far away from D that the arm couldn't reach D. The second condition ensures that B won't be so close to point D that the angle BCD becomes acute (because the servo cannot rotate more than 90 degrees to the right).

B is on a circle whose center is the origin and radius is l<sub>1</sub>, so its coordinates can be expressed as follow:

<img src="http://latex.codecogs.com/svg.latex?\large&space;m&space;\in&space;[-l_1,&space;l_1]&space;\\\\&space;n&space;=&space;\sqrt{l_1^2&space;-&space;m^2}" title="\large m \in [-l_1, l_1] \\\\ n = \sqrt{l_1^2 - m^2}" />

So the inequality could be formulated:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\sqrt{l_2^2&space;&plus;&space;l_3^2}&space;<&space;\sqrt{(m&space;-&space;a)^2&space;&plus;&space;\left&space;(&space;\sqrt{l_1^2&space;-&space;m^2}&space;-&space;b&space;\right&space;)^2}&space;<&space;l_2^2&space;&plus;&space;l_3^2" title="\large \sqrt{l_2^2 + l_3^2} < \sqrt{(m - a)^2 + \left ( \sqrt{l_1^2 - m^2} - b \right )^2} < l_2^2 + l_3^2" />

Any m that satisfies this inequality will be a solution. Given a particular m, the angles of servos could be calculated through a series of geometric derivations

<img src="demo/angle-solution.svg"/>

<br/>

<img src="http://latex.codecogs.com/svg.latex?\large&space;\theta_1&space;=&space;\arcsin{\frac{m}{l_1}}" title="\large \theta_1 = \arcsin{\frac{m}{l_1}}" />
<br/>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\theta_2&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;-&space;\alpha&space;-&space;\beta" title="\large \theta_2 = \frac{\pi}{2} - \theta_1 - \alpha - \beta" />
<br/>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\theta_3&space;=&space;\pi&space;-&space;\widehat{BCD}" title="\large \theta_3 = \pi - \widehat{BCD}" />

&alpha; could be obtained by using the cosine rule

<img src="http://latex.codecogs.com/svg.latex?\large&space;\alpha&space;=&space;\arccos{\left&space;(&space;\frac{l_2^2&space;&plus;&space;c^2&space;-&space;l_3^2&space;}{2l_2^2c}&space;\right&space;)}" title="\large \alpha = \arccos{\left ( \frac{l_2^2 + c^2 - l_3^2 }{2l_2^2c} \right )}" />

where c is the length of BD

<img src="http://latex.codecogs.com/svg.latex?\large&space;c&space;=&space;\sqrt{(a-m)^2&space;&plus;&space;(b&space;-&space;n)^2}" title="\large c = \sqrt{(a-m)^2 + (b - n)^2}" />
<br/>

<img src="http://latex.codecogs.com/svg.latex?\large&space;n&space;=&space;\sqrt{l_1^2&space;-&space;m^2}" title="\large n = \sqrt{l_1^2 - m^2}" />

&beta; can be found by applying arctan to the gradient of BD:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\beta&space;=&space;\arctan{\frac{b-n}{a-m}}" title="\large \beta = \arctan{\frac{b-n}{a-m}}" />

angle BCD can be found by using the cosine rule:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\widehat{BCD}&space;=&space;\arccos{\left&space;(&space;\frac{l_2^2&plus;l_3^2&space;-&space;c^2}{2l_2l_3}&space;\right&space;)}" title="\large \widehat{BCD} = \arccos{\left ( \frac{l_2^2+l_3^2 - c^2}{2l_2l_3} \right )}" />

Finally, combine all equations and using only known values:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\theta_1&space;=&space;\arcsin{\frac{m}{l_1}}\\\\\\&space;\theta_2&space;=&space;\frac{\pi}{2}&space;-&space;\arcsin{\frac{m}{l_1}}&space;-&space;\arccos{\left&space;(&space;\frac{l_2^2&space;&plus;&space;(a-m)^2&space;&plus;(b-\sqrt{l_1^2&space;-&space;m^2})^2&space;-&space;l_3^2&space;}{2l_2^2&space;\sqrt{(a-m)^2&space;&plus;&space;(b-\sqrt{l_1^2&space;-&space;m^2})^2}}&space;\right&space;)}&space;-&space;\arctan{\frac{b-\sqrt{l_1^2&space;-&space;m^2}}{a-m}}\\\\\\&space;\theta_3&space;=&space;\pi&space;-&space;\arccos{\left&space;(&space;\frac{l_2^2&plus;l_3^2&space;-&space;(a-m)^2-(b-\sqrt{l_1^2&space;-&space;m^2})^2}{2l_2l_3}&space;\right&space;)}" title="\large \theta_1 = \arcsin{\frac{m}{l_1}}\\\\\\ \theta_2 = \frac{\pi}{2} - \arcsin{\frac{m}{l_1}} - \arccos{\left ( \frac{l_2^2 + (a-m)^2 +(b-\sqrt{l_1^2 - m^2})^2 - l_3^2 }{2l_2^2 \sqrt{(a-m)^2 + (b-\sqrt{l_1^2 - m^2})^2}} \right )} - \arctan{\frac{b-\sqrt{l_1^2 - m^2}}{a-m}}\\\\\\ \theta_3 = \pi - \arccos{\left ( \frac{l_2^2+l_3^2 - (a-m)^2-(b-\sqrt{l_1^2 - m^2})^2}{2l_2l_3} \right )}" />

The spatial coordinate of each joint could be recovered from &phi;, &theta;<sub>1</sub>, &theta;<sub>2</sub> and &theta;<sub>3</sub>:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\gamma_1&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;\\&space;\gamma_2&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;-&space;\theta_2&space;\\&space;\gamma_3&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;-&space;\theta_2&space;-&space;\theta_3&space;\\\\&space;A&space;(0,&space;0,&space;0)&space;\\&space;B&space;(l_1\cos{\gamma_1}\cos{\varphi},\&space;l_1\cos{\gamma_1}\sin{\varphi},\&space;l_1\sin{\gamma_1})\\&space;C&space;(l_2\cos{\gamma_2}\cos{\varphi}&space;&plus;&space;X_B,\&space;l_2\cos{\gamma_2}\sin{\varphi}&space;&plus;&space;Y_B,\&space;l_2\sin{\gamma_2}&space;&plus;&space;Z_B)\\&space;D&space;(l_3\cos{\gamma_3}\cos{\varphi}&space;&plus;&space;X_C,\&space;l_3\cos{\gamma_3}\sin{\varphi}&space;&plus;&space;Y_C,\&space;l_3\sin{\gamma_3}&space;&plus;&space;Z_C)" title="\large \gamma_1 = \frac{\pi}{2} - \theta_1 \\ \gamma_2 = \frac{\pi}{2} - \theta_1 - \theta_2 \\ \gamma_3 = \frac{\pi}{2} - \theta_1 - \theta_2 - \theta_3 \\\\ A (0, 0, 0) \\ B (l_1\cos{\gamma_1}\cos{\varphi},\ l_1\cos{\gamma_1}\sin{\varphi},\ l_1\sin{\gamma_1})\\ C (l_2\cos{\gamma_2}\cos{\varphi} + X_B,\ l_2\cos{\gamma_2}\sin{\varphi} + Y_B,\ l_2\sin{\gamma_2} + Z_B)\\ D (l_3\cos{\gamma_3}\cos{\varphi} + X_C,\ l_3\cos{\gamma_3}\sin{\varphi} + Y_C,\ l_3\sin{\gamma_3} + Z_C)" />


