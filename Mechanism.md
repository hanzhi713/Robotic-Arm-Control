# Mechanism

To reach point D in space, the robotic arm first needs to rotate <img src="http://latex.codecogs.com/svg.latex?\inline&space;\large&space;\varphi" title="\large \varphi" /> degrees so that the linkages of the arm and the target point are coplanar (i.e. they all line on the blue plane). 

<img src="http://latex.codecogs.com/svg.latex?\large&space;\varphi&space;=&space;\arctan{\frac{y}{x}}" title="\large \varphi = \arctan{\frac{y}{x}}" />

<img src="demo/three-to-two.png" width="600px">

The spatial coordinate of the target point D is mapped to a coordinate on the blue plane, which could be calculated as follow

<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;f:&space;\mathbb{R}^3&space;\rightarrow&space;\mathbb{R}^2&space;\\&space;f(x,&space;y,&space;z)&space;=&space;(\sqrt{x^2&space;&plus;&space;y^2},\&space;z)&space;\end{array}" title="\large \large \begin{array}{l} f: \mathbb{R}^3 \rightarrow \mathbb{R}^2 \\ f(x, y, z) = (\sqrt{x^2 + y^2},\ z) \end{array}" />

Consequently, this problem is simplied to a two dimensional route planning problem. 

<img src="demo/2d.svg" width="500px">

As shown in the diagram above, the solution for this problem is not unique. A particular position of the first segment (l<sub>1</sub>) corresponds to a particular solution. However, not all positions of point B are suitable. We require:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\left\{&space;\begin{array}{ll}&space;&l_2&space;&plus;&space;l_3&space;>&space;BD&space;\\&space;\\&space;&\sqrt{l_2^2&space;&plus;&space;l_3^2}&space;<&space;BD&space;\end{array}&space;\right." title="\large \large \left\{ \begin{array}{ll} &l_2 + l_3 > BD \\ \\ &\sqrt{l_2^2 + l_3^2} < BD \end{array} \right." />

The first condition ensures that point B won't be so far away from D that the arm couldn't reach D. The second condition ensures that B won't be so close to point D that the angle BCD becomes acute (because the servo cannot rotate more than 90 degrees to the right).

B is on a circle whose center is the origin and radius is l<sub>1</sub>, so its coordinates can be expressed as follow:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;m&space;\in&space;[-l_1,&space;l_1]&space;\\\\&space;n&space;=&space;\sqrt{l_1^2&space;-&space;m^2}&space;\end{array}" title="\large \large \begin{array}{l} m \in [-l_1, l_1] \\\\ n = \sqrt{l_1^2 - m^2} \end{array}" />

So the inequality could be formulated:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\sqrt{l_2^2&space;&plus;&space;l_3^2}&space;<&space;\sqrt{(m&space;-&space;a)^2&space;&plus;&space;\left&space;(&space;\sqrt{l_1^2&space;-&space;m^2}&space;-&space;b&space;\right&space;)^2}&space;<&space;l_2&space;&plus;&space;l_3" title="\large \sqrt{l_2^2 + l_3^2} < \sqrt{(m - a)^2 + \left ( \sqrt{l_1^2 - m^2} - b \right )^2} < l_2 + l_3" />

Any m that satisfies this inequality will be a solution. Given a particular m, the angles of servos could be calculated through a series of geometric derivations.

## Start interlude

Well, there is another more straightforward way to calculate the angle and the coordinates.

Suppose the angle subtended by the x-axis and DO is &phi; and the distance DO equals k. We can write the above formula into such:
<p>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;l_2^2&plus;l_3^2&space;\leq(k\cos{\varphi}-l_1\sin{\theta_1})^2&plus;(k\sin{\varphi}-l_1\cos{\theta_1})^2\leq&space;(l_2&plus;l_3)^2\\\\&space;\frac{k^2&plus;l_1^2-(l_2&plus;l_3)^2}{2kl_1}&space;\leq{sin(\theta_1&plus;\varphi)\leq{\frac{k^2&plus;l_1^2-l_2^2-l_3^2}{2kl_1}}}&space;\end{array}" title="\large \large \begin{array}{l} l_2^2+l_3^2 \leq(k\cos{\varphi}-l_1\sin{\theta_1})^2+(k\sin{\varphi}-l_1\cos{\theta_1})^2\leq (l_2+l_3)^2\\\\ \frac{k^2+l_1^2-(l_2+l_3)^2}{2kl_1} \leq{sin(\theta_1+\varphi)\leq{\frac{k^2+l_1^2-l_2^2-l_3^2}{2kl_1}}} \end{array}" />
</p>
<p>
At this point, naturally one will think of using the arcsin to solve the inequality. However, we have to make sure that the left side of the inequality is no smaller than -1, and the right side of the inequality is no larger than 1. Besides, as:
</p>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}&space;{lcr}&space;\theta_1\in[-90\degree,&space;90\degree]&space;&&space;\varphi\in[0\degree,&space;90\degree]&space;&&space;\theta_1&plus;\varphi\in[-90\degree,&space;180\degree]&space;\end{array}" title="\large \large \begin{array} {lcr} \theta_1\in[-90\degree, 90\degree] & \varphi\in[0\degree, 90\degree] & \theta_1+\varphi\in[-90\degree, 180\degree] \end{array}" />
<p>
we have to make sure the angles are in the correct intervals.

Let's suppose the above inequality is in the interval [-1, 1]. Then
<br/>
<br/>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\arcsin\left(\frac{k^2&plus;l_1^2-(l_2&plus;l_3)^2}{2kl_1}\right)&space;\leq\theta_1&plus;\varphi&space;\leq\arcsin\left(\frac{k^2&plus;l_1^2-l_2^2-l_3^2}{2kl_1}\right)" title="\large \arcsin\left(\frac{k^2+l_1^2-(l_2+l_3)^2}{2kl_1}\right) \leq\theta_1+\varphi \leq\arcsin\left(\frac{k^2+l_1^2-l_2^2-l_3^2}{2kl_1}\right)" />
</p>
<p>
Let
<br/>
<br/>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\begin{array}{lr}&space;\delta_1=\arcsin\left(\frac{k^2&plus;l_1^2-(l_2&plus;l_3)^2}{2kl_1}\right)&space;&&space;\delta_2&space;=&space;\arcsin\left(\frac{k^2&plus;l_1^2-l_2^2-l_3^2}{2kl_1}\right)&space;\end{array}" title="\large \begin{array}{lr} \delta_1=\arcsin\left(\frac{k^2+l_1^2-(l_2+l_3)^2}{2kl_1}\right) & \delta_2 = \arcsin\left(\frac{k^2+l_1^2-l_2^2-l_3^2}{2kl_1}\right) \end{array}" />
<br/>
<br/>
We also need to check whether 180 - &delta;1 and 180 - &delta;2 is also an interval for &theta;1 + &phi;. To make sure that everything is proper, we need to check where 90 + &phi; lies inside the interval. 90 + &phi; is a limit that cannot be exceeded. Nevertheless, using this way we can find the range of &theta;1, and from then on we can easily find other angles.
</p>

## End interlude

<img src="demo/angle-solution.svg"/>
<br/>

<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;\theta_1&space;=&space;\arcsin{\frac{m}{l_1}}&space;\\&space;\\&space;\theta_2&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;-&space;\alpha&space;-&space;\beta&space;\\&space;\\&space;\theta_3&space;=&space;\pi&space;-&space;B\widehat{C}D&space;\end{array}" title="\large \large \begin{array}{l} \theta_1 = \arcsin{\frac{m}{l_1}} \\ \\ \theta_2 = \frac{\pi}{2} - \theta_1 - \alpha - \beta \\ \\ \theta_3 = \pi - B\widehat{C}D \end{array}" />

&alpha; could be obtained by using the cosine rule

<img src="http://latex.codecogs.com/svg.latex?\large&space;\alpha&space;=&space;\arccos{\left&space;(&space;\frac{l_2^2&space;&plus;&space;c^2&space;-&space;l_3^2&space;}{2l_2^2c}&space;\right&space;)}" title="\large \alpha = \arccos{\left ( \frac{l_2^2 + c^2 - l_3^2 }{2l_2^2c} \right )}" />

where c is the length of BD

<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;c&space;=&space;\sqrt{(a-m)^2&space;&plus;&space;(b-n)^2}&space;\\\\&space;n&space;=&space;\sqrt{l_1^2&space;-&space;m^2}&space;\end{array}" title="\large \large \begin{array}{l} c = \sqrt{(a-m)^2 + (b-n)^2} \\\\ n = \sqrt{l_1^2 - m^2} \end{array}" />

&beta; can be found by applying arctan to the gradient of BD:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\beta&space;=&space;\arctan{\frac{b-n}{a-m}}" title="\large \beta = \arctan{\frac{b-n}{a-m}}" />

angle BCD can be found by using the cosine rule:

<img src="http://latex.codecogs.com/svg.latex?\large&space;B\widehat{C}D&space;=&space;\arccos{\left&space;(&space;\frac{l_2^2&plus;l_3^2&space;-&space;c^2}{2l_2l_3}&space;\right&space;)}" title="\large B\widehat{C}D = \arccos{\left ( \frac{l_2^2+l_3^2 - c^2}{2l_2l_3} \right )}" />

Finally, combine all equations and using only known values:

<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;\theta_1&space;=&space;\arcsin{\frac{m}{l_1}}\\&space;\theta_2&space;=&space;\frac{\pi}{2}&space;-&space;\arcsin{\frac{m}{l_1}}&space;-&space;\arccos{\left&space;(&space;\frac{l_2^2&space;&plus;&space;(a-m)^2&space;&plus;(b-\sqrt{l_1^2&space;-&space;m^2})^2&space;-&space;l_3^2&space;}{2l_2^2&space;\sqrt{(a-m)^2&space;&plus;&space;(b-\sqrt{l_1^2&space;-&space;m^2})^2}}&space;\right&space;)}&space;-&space;\arctan{\frac{b-\sqrt{l_1^2&space;-&space;m^2}}{a-m}}\\&space;\theta_3&space;=&space;\pi&space;-&space;\arccos{\left&space;(&space;\frac{l_2^2&plus;l_3^2&space;-&space;(a-m)^2-(b-\sqrt{l_1^2&space;-&space;m^2})^2}{2l_2l_3}&space;\right&space;)}&space;\end{array}" title="\large \large \begin{array}{l} \theta_1 = \arcsin{\frac{m}{l_1}}\\ \theta_2 = \frac{\pi}{2} - \arcsin{\frac{m}{l_1}} - \arccos{\left ( \frac{l_2^2 + (a-m)^2 +(b-\sqrt{l_1^2 - m^2})^2 - l_3^2 }{2l_2^2 \sqrt{(a-m)^2 + (b-\sqrt{l_1^2 - m^2})^2}} \right )} - \arctan{\frac{b-\sqrt{l_1^2 - m^2}}{a-m}}\\ \theta_3 = \pi - \arccos{\left ( \frac{l_2^2+l_3^2 - (a-m)^2-(b-\sqrt{l_1^2 - m^2})^2}{2l_2l_3} \right )} \end{array}" />

The spatial coordinate of each joint could be recovered from &phi;, &theta;<sub>1</sub>, &theta;<sub>2</sub> and &theta;<sub>3</sub>:
<p>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}&space;{l}&space;\gamma_1&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;\\\\&space;\gamma_2&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;-&space;\theta_2&space;\\\\&space;\gamma_3&space;=&space;\frac{\pi}{2}&space;-&space;\theta_1&space;-&space;\theta_2&space;-&space;\theta_3\\\\\\&space;\end{array}" title="\large \large \begin{array} {l} \gamma_1 = \frac{\pi}{2} - \theta_1 \\\\ \gamma_2 = \frac{\pi}{2} - \theta_1 - \theta_2 \\\\ \gamma_3 = \frac{\pi}{2} - \theta_1 - \theta_2 - \theta_3\\\\\\ \end{array}" />
</p>
<img src="http://latex.codecogs.com/svg.latex?\large&space;\large&space;\begin{array}{l}&space;A&space;(0,&space;0,&space;0)&space;\\\\&space;B&space;(l_1\cos{\gamma_1}\cos{\varphi},\&space;l_1\cos{\gamma_1}\sin{\varphi},\&space;l_1\sin{\gamma_1})\\\\&space;C&space;(l_2\cos{\gamma_2}\cos{\varphi}&space;&plus;&space;X_B,\&space;l_2\cos{\gamma_2}\sin{\varphi}&space;&plus;&space;Y_B,\&space;l_2\sin{\gamma_2}&space;&plus;&space;Z_B)\\\\&space;D&space;(l_3\cos{\gamma_3}\cos{\varphi}&space;&plus;&space;X_C,\&space;l_3\cos{\gamma_3}\sin{\varphi}&space;&plus;&space;Y_C,\&space;l_3\sin{\gamma_3}&space;&plus;&space;Z_C)&space;\end{array}" title="\large \large \begin{array}{l} A (0, 0, 0) \\\\ B (l_1\cos{\gamma_1}\cos{\varphi},\ l_1\cos{\gamma_1}\sin{\varphi},\ l_1\sin{\gamma_1})\\\\ C (l_2\cos{\gamma_2}\cos{\varphi} + X_B,\ l_2\cos{\gamma_2}\sin{\varphi} + Y_B,\ l_2\sin{\gamma_2} + Z_B)\\\\ D (l_3\cos{\gamma_3}\cos{\varphi} + X_C,\ l_3\cos{\gamma_3}\sin{\varphi} + Y_C,\ l_3\sin{\gamma_3} + Z_C) \end{array}" />

