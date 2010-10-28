/* This code is part of homest and was automatically generated by maple */

#include "maplefuncs.h"

void calc2DHomogLinCoeffs2(double m1[2],double m2[2],double eq[18])
{
  double t1;
  double t2;
  double t3;
  double t4;
  {
    t1 = m1[0];
    eq[0] = -t1;
    t2 = m1[1];
    eq[1] = -t2;
    eq[2] = -1.0;
    eq[3] = 0.0;
    eq[4] = 0.0;
    eq[5] = 0.0;
    t3 = m2[0];
    eq[6] = t3*t1;
    eq[7] = t2*t3;
    eq[8] = t3;
    eq[9] = 0.0;
    eq[10] = 0.0;
    eq[11] = 0.0;
    eq[12] = eq[0];
    eq[13] = eq[1];
    eq[14] = -1.0;
    t4 = m2[1];
    eq[15] = t4*t1;
    eq[16] = t4*t2;
    eq[17] = t4;
    return;
  }
}

void calc2DAffHomogLinCoeffs2(double m1[2],double m2[2],double eq[12],
double rh[2])
{
  {
    eq[0] = -m1[0];
    eq[1] = -m1[1];
    eq[2] = -1.0;
    eq[3] = 0.0;
    eq[4] = 0.0;
    eq[5] = 0.0;
    eq[6] = 0.0;
    eq[7] = 0.0;
    eq[8] = 0.0;
    eq[9] = eq[0];
    eq[10] = eq[1];
    eq[11] = -1.0;
    rh[0] = -m2[0];
    rh[1] = -m2[1];
    return;
  }
}

void calc2DHomogLinCoeffs3(double n1[3],double n2[3],double eq[27])
{
  double t1;
  double t10;
  double t11;
  double t12;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  {
    eq[0] = 0.0;
    eq[1] = 0.0;
    eq[2] = 0.0;
    t1 = n2[2];
    t2 = n1[0];
    t3 = t2*t1;
    eq[3] = -t3;
    t4 = n1[1];
    t5 = t4*t1;
    eq[4] = -t5;
    t6 = n1[2];
    t7 = t1*t6;
    eq[5] = -t7;
    t8 = n2[1];
    eq[6] = t8*t2;
    eq[7] = t8*t4;
    eq[8] = t8*t6;
    eq[9] = t3;
    eq[10] = t5;
    eq[11] = t7;
    eq[12] = 0.0;
    eq[13] = 0.0;
    eq[14] = 0.0;
    t9 = n2[0];
    t10 = t9*t2;
    eq[15] = -t10;
    t11 = t9*t4;
    eq[16] = -t11;
    t12 = t9*t6;
    eq[17] = -t12;
    eq[18] = -eq[6];
    eq[19] = -eq[7];
    eq[20] = -eq[8];
    eq[21] = t10;
    eq[22] = t11;
    eq[23] = t12;
    eq[24] = 0.0;
    eq[25] = 0.0;
    eq[26] = 0.0;
    return;
  }
}

#include <math.h>
void calc2DHomogTransfer(double m1[2],double h[9],double xm1[2])
{
  double t15;
  double t2;
  double t5;
  {
    t2 = m1[0];
    t5 = m1[1];
    t15 = 1/(h[6]*t2+h[7]*t5+h[8]);
    xm1[0] = (h[0]*t2+h[1]*t5+h[2])*t15;
    xm1[1] = (h[3]*t2+h[4]*t5+h[5])*t15;
    return;
  }
}

void calc2DHomogTransferJac(double m1[2],double h[9],double xm1_jac[2][9])
{
  double t1;
  double t16;
  double t17;
  double t18;
  double t27;
  double t5;
  double t8;
  double t9;
  {
    t1 = m1[0];
    t5 = m1[1];
    t8 = h[6]*t1+h[7]*t5+h[8];
    t9 = 1/t8;
    xm1_jac[0][0] = t9*t1;
    xm1_jac[0][1] = t5*t9;
    xm1_jac[0][2] = t9;
    xm1_jac[0][3] = 0.0;
    xm1_jac[0][4] = 0.0;
    xm1_jac[0][5] = 0.0;
    t16 = t8*t8;
    t17 = 1/t16;
    t18 = (h[0]*t1+h[1]*t5+h[2])*t17;
    xm1_jac[0][6] = -t18*t1;
    xm1_jac[0][7] = -t18*t5;
    xm1_jac[0][8] = -t18;
    xm1_jac[1][0] = 0.0;
    xm1_jac[1][1] = 0.0;
    xm1_jac[1][2] = 0.0;
    xm1_jac[1][3] = xm1_jac[0][0];
    xm1_jac[1][4] = xm1_jac[0][1];
    xm1_jac[1][5] = xm1_jac[0][2];
    t27 = (h[3]*t1+h[4]*t5+h[5])*t17;
    xm1_jac[1][6] = -t27*t1;
    xm1_jac[1][7] = -t27*t5;
    xm1_jac[1][8] = -t27;
    return;
  }
}

#include <math.h>
void calc2DHomogSymTransfer(double m1[2],double m2[2],double h[9],
double xm12[4])
{
  double t1;
  double t11;
  double t13;
  double t14;
  double t15;
  double t17;
  double t18;
  double t2;
  double t20;
  double t21;
  double t23;
  double t26;
  double t28;
  double t34;
  double t4;
  double t5;
  double t53;
  double t66;
  double t68;
  double t74;
  double t8;
  double t9;
  {
    t1 = h[4];
    t2 = h[8];
    t4 = h[5];
    t5 = h[7];
    t8 = h[0];
    t9 = t8*t1;
    t11 = t8*t4;
    t13 = h[3];
    t14 = h[1];
    t15 = t14*t13;
    t17 = h[2];
    t18 = t13*t17;
    t20 = h[6];
    t21 = t20*t14;
    t23 = t20*t17;
    t26 = 1/(t9*t2-t11*t5-t15*t2+t18*t5+t21*t4-t23*t1);
    t28 = m2[0];
    t34 = m2[1];
    t53 = 1/(-(-t13*t5+t1*t20)*t26*t28+(-t8*t5+t21)*t26*t34+(t9-t15)*t26);
    xm12[0] = (-(-t2*t1+t4*t5)*t26*t28+(-t14*t2+t17*t5)*t26*t34+(t14*t4-t17*t1)
*t26)*t53;
    xm12[1] = ((-t2*t13+t4*t20)*t26*t28-(-t2*t8+t23)*t26*t34-(t11-t18)*t26)*t53
;
    t66 = m1[0];
    t68 = m1[1];
    t74 = 1/(t20*t66+t5*t68+t2);
    xm12[2] = (t8*t66+t14*t68+t17)*t74;
    xm12[3] = (t13*t66+t1*t68+t4)*t74;
    return;
  }
}

void calc2DHomogSymTransferJac(double m1[2],double m2[2],double h[9],
double xm12_jac[4][9])
{
  double t1;
  double t100;
  double t102;
  double t103;
  double t105;
  double t11;
  double t113;
  double t115;
  double t117;
  double t123;
  double t125;
  double t126;
  double t127;
  double t129;
  double t13;
  double t131;
  double t133;
  double t134;
  double t136;
  double t138;
  double t14;
  double t142;
  double t145;
  double t147;
  double t149;
  double t15;
  double t155;
  double t159;
  double t161;
  double t163;
  double t165;
  double t169;
  double t17;
  double t171;
  double t173;
  double t174;
  double t176;
  double t177;
  double t178;
  double t18;
  double t180;
  double t181;
  double t186;
  double t188;
  double t191;
  double t193;
  double t195;
  double t2;
  double t20;
  double t201;
  double t203;
  double t205;
  double t207;
  double t208;
  double t21;
  double t210;
  double t211;
  double t221;
  double t223;
  double t23;
  double t25;
  double t26;
  double t27;
  double t274;
  double t276;
  double t278;
  double t279;
  double t28;
  double t283;
  double t284;
  double t285;
  double t29;
  double t291;
  double t30;
  double t31;
  double t35;
  double t36;
  double t37;
  double t38;
  double t4;
  double t42;
  double t43;
  double t48;
  double t49;
  double t5;
  double t53;
  double t56;
  double t58;
  double t59;
  double t67;
  double t68;
  double t69;
  double t7;
  double t70;
  double t72;
  double t73;
  double t74;
  double t76;
  double t77;
  double t79;
  double t8;
  double t83;
  double t84;
  double t86;
  double t87;
  double t88;
  double t9;
  double t90;
  double t95;
  double t96;
  double t98;
  {
    t1 = h[4];
    t2 = h[8];
    t4 = h[5];
    t5 = h[7];
    t7 = -t2*t1+t4*t5;
    t8 = h[0];
    t9 = t8*t1;
    t11 = t8*t4;
    t13 = h[3];
    t14 = h[1];
    t15 = t14*t13;
    t17 = h[2];
    t18 = t13*t17;
    t20 = h[6];
    t21 = t20*t14;
    t23 = t20*t17;
    t25 = t9*t2-t11*t5-t15*t2+t18*t5+t21*t4-t23*t1;
    t26 = t25*t25;
    t27 = 1/t26;
    t28 = t7*t27;
    t29 = m2[0];
    t30 = -t7;
    t31 = t29*t30;
    t35 = -t14*t2+t17*t5;
    t36 = t35*t27;
    t37 = m2[1];
    t38 = t37*t30;
    t42 = t14*t4-t17*t1;
    t43 = t42*t27;
    t48 = -t13*t5+t1*t20;
    t49 = 1/t25;
    t53 = -t8*t5+t21;
    t56 = t9-t15;
    t58 = -t48*t49*t29+t53*t49*t37+t56*t49;
    t59 = 1/t58;
    t67 = t58*t58;
    t68 = 1/t67;
    t69 = (-t7*t49*t29+t35*t49*t37+t42*t49)*t68;
    t70 = t48*t27;
    t72 = t5*t49;
    t73 = t72*t37;
    t74 = t53*t27;
    t76 = t1*t49;
    t77 = t56*t27;
    t79 = t70*t31-t73-t74*t38+t76-t77*t30;
    xm12_jac[0][0] = (t28*t31-t36*t38-t43*t30)*t59-t69*t79;
    t83 = -t2*t13+t4*t20;
    t84 = t29*t83;
    t86 = t2*t49;
    t87 = t86*t37;
    t88 = t37*t83;
    t90 = t4*t49;
    t95 = t20*t49;
    t96 = t95*t37;
    t98 = t13*t49;
    t100 = t70*t84+t96-t74*t88-t98-t77*t83;
    xm12_jac[0][1] = (t28*t84-t87-t36*t88+t90-t43*t83)*t59-t69*t100;
    t102 = -t48;
    t103 = t29*t102;
    t105 = t37*t102;
    t113 = t70*t103-t74*t105-t77*t102;
    xm12_jac[0][2] = (t28*t103+t73-t36*t105-t76-t43*t102)*t59-t69*t113;
    t115 = t29*t35;
    t117 = t35*t35;
    t123 = t72*t29;
    t125 = t37*t35;
    t126 = t74*t125;
    t127 = t14*t49;
    t129 = t123+t70*t115-t126-t127-t77*t35;
    xm12_jac[0][3] = (t28*t115-t117*t27*t37-t43*t35)*t59-t69*t129;
    t131 = t86*t29;
    t133 = t2*t8-t23;
    t134 = t29*t133;
    t136 = t37*t133;
    t138 = t17*t49;
    t142 = t95*t29;
    t145 = t8*t49;
    t147 = -t142+t70*t134-t74*t136+t145-t77*t133;
    xm12_jac[0][4] = (t131+t28*t134-t36*t136-t138-t43*t133)*t59-t69*t147;
    t149 = t29*t53;
    t155 = t53*t53;
    t159 = t70*t149-t155*t27*t37-t77*t53;
    xm12_jac[0][5] = (-t123+t28*t149-t126+t127-t43*t53)*t59-t69*t159;
    t161 = t29*t42;
    t163 = t37*t42;
    t165 = t42*t42;
    t169 = t76*t29;
    t171 = t127*t37;
    t173 = t77*t42;
    t174 = -t169+t70*t161+t171-t74*t163-t173;
    xm12_jac[0][6] = (t28*t161-t36*t163-t165*t27)*t59-t69*t174;
    t176 = t90*t29;
    t177 = -t11+t18;
    t178 = t29*t177;
    t180 = t138*t37;
    t181 = t37*t177;
    t186 = t98*t29;
    t188 = t145*t37;
    t191 = t186+t70*t178-t188-t74*t181-t77*t177;
    xm12_jac[0][7] = (-t176+t28*t178+t180-t36*t181-t43*t177)*t59-t69*t191;
    t193 = t29*t56;
    t195 = t37*t56;
    t201 = t56*t56;
    t203 = t70*t193-t74*t195-t201*t27;
    xm12_jac[0][8] = (t169+t28*t193-t171-t36*t195-t173)*t59-t69*t203;
    t205 = t83*t27;
    t207 = -t133;
    t208 = t207*t27;
    t210 = -t177;
    t211 = t210*t27;
    t221 = (t83*t49*t29-t207*t49*t37-t210*t49)*t68;
    xm12_jac[1][0] = (-t205*t31+t87+t208*t38-t90+t211*t30)*t59-t221*t79;
    t223 = t83*t83;
    xm12_jac[1][1] = (-t223*t27*t29+t208*t88+t211*t83)*t59-t221*t100;
    xm12_jac[1][2] = (-t205*t103-t96+t208*t105+t98+t211*t102)*t59-t113*t221;
    xm12_jac[1][3] = (-t131-t205*t115+t208*t125+t138+t211*t35)*t59-t221*t129;
    xm12_jac[1][4] = (-t205*t134+t208*t136+t211*t133)*t59-t221*t147;
    xm12_jac[1][5] = (t142-t205*t149+t208*t37*t53-t145+t211*t53)*t59-t221*t159;
    xm12_jac[1][6] = (t176-t205*t161-t180+t208*t163+t211*t42)*t59-t221*t174;
    xm12_jac[1][7] = (-t205*t178+t208*t181+t211*t177)*t59-t221*t191;
    xm12_jac[1][8] = (-t186-t205*t193+t188+t208*t195+t211*t56)*t59-t221*t203;
    t274 = m1[0];
    t276 = m1[1];
    t278 = t20*t274+t5*t276+t2;
    t279 = 1/t278;
    xm12_jac[2][0] = t274*t279;
    xm12_jac[2][1] = t276*t279;
    xm12_jac[2][2] = t279;
    xm12_jac[2][3] = 0.0;
    xm12_jac[2][4] = 0.0;
    xm12_jac[2][5] = 0.0;
    t283 = t278*t278;
    t284 = 1/t283;
    t285 = (t8*t274+t14*t276+t17)*t284;
    xm12_jac[2][6] = -t285*t274;
    xm12_jac[2][7] = -t285*t276;
    xm12_jac[2][8] = -t285;
    xm12_jac[3][0] = 0.0;
    xm12_jac[3][1] = 0.0;
    xm12_jac[3][2] = 0.0;
    xm12_jac[3][3] = xm12_jac[2][0];
    xm12_jac[3][4] = xm12_jac[2][1];
    xm12_jac[3][5] = xm12_jac[2][2];
    t291 = (t13*t274+t1*t276+t4)*t284;
    xm12_jac[3][6] = -t291*t274;
    xm12_jac[3][7] = -t291*t276;
    xm12_jac[3][8] = -t291;
    return;
  }
}

#include <math.h>
void calc2DHomogSampsonErr(double m1[2],double m2[2],double h[9],double err[1])
{
  double t1;
  double t10;
  double t100;
  double t104;
  double t108;
  double t112;
  double t118;
  double t12;
  double t122;
  double t125;
  double t126;
  double t129;
  double t13;
  double t139;
  double t14;
  double t141;
  double t144;
  double t15;
  double t150;
  double t153;
  double t161;
  double t167;
  double t17;
  double t174;
  double t18;
  double t19;
  double t193;
  double t199;
  double t2;
  double t20;
  double t201;
  double t202;
  double t21;
  double t213;
  double t219;
  double t22;
  double t220;
  double t222;
  double t225;
  double t23;
  double t236;
  double t24;
  double t243;
  double t250;
  double t253;
  double t26;
  double t260;
  double t27;
  double t271;
  double t273;
  double t28;
  double t29;
  double t296;
  double t3;
  double t30;
  double t303;
  double t31;
  double t317;
  double t33;
  double t331;
  double t335;
  double t339;
  double t34;
  double t342;
  double t345;
  double t35;
  double t350;
  double t354;
  double t36;
  double t361;
  double t365;
  double t37;
  double t374;
  double t39;
  double t4;
  double t40;
  double t41;
  double t42;
  double t43;
  double t44;
  double t45;
  double t46;
  double t47;
  double t49;
  double t51;
  double t57;
  double t6;
  double t65;
  double t66;
  double t68;
  double t69;
  double t7;
  double t72;
  double t78;
  double t8;
  double t86;
  double t87;
  double t90;
  double t95;
  {
    t1 = m2[0];
    t2 = h[6];
    t3 = t2*t1;
    t4 = m1[0];
    t6 = h[7];
    t7 = t1*t6;
    t8 = m1[1];
    t10 = h[8];
    t12 = h[0];
    t13 = t12*t4;
    t14 = h[1];
    t15 = t14*t8;
    t17 = t3*t4+t7*t8+t1*t10-t13-t15-h[2];
    t18 = m2[1];
    t19 = t18*t18;
    t20 = t2*t2;
    t21 = t19*t20;
    t22 = t18*t2;
    t23 = h[3];
    t24 = t23*t22;
    t26 = t23*t23;
    t27 = t6*t6;
    t28 = t19*t27;
    t29 = t18*t6;
    t30 = h[4];
    t31 = t29*t30;
    t33 = t30*t30;
    t34 = t4*t4;
    t35 = t20*t34;
    t36 = t2*t4;
    t37 = t6*t8;
    t39 = 2.0*t36*t37;
    t40 = t36*t10;
    t41 = 2.0*t40;
    t42 = t8*t8;
    t43 = t42*t27;
    t44 = t37*t10;
    t45 = 2.0*t44;
    t46 = t10*t10;
    t47 = t21-2.0*t24+t26+t28-2.0*t31+t33+t35+t39+t41+t43+t45+t46;
    t49 = t12*t12;
    t51 = t6*t30;
    t57 = t20*t2;
    t65 = t1*t1;
    t66 = t65*t20;
    t68 = t65*t57;
    t69 = t4*t10;
    t72 = t2*t49;
    t78 = t27*t6;
    t86 = t65*t78;
    t87 = t8*t10;
    t90 = t65*t27;
    t95 = -2.0*t49*t18*t51-2.0*t3*t12*t46-2.0*t1*t57*t12*t34-2.0*t3*t12*t33+t66
*t43+2.0*t68*t69+2.0*t72*t69-2.0*t7*t14*t46-2.0*t1*t78*t14*t42-2.0*t7*t14*t26+
2.0*t86*t87+t90*t35+2.0*t49*t6*t87;
    t100 = t14*t14;
    t104 = t100*t2;
    t108 = t2*t23;
    t112 = t78*t42*t8;
    t118 = t57*t34*t4;
    t122 = t10*t26;
    t125 = t57*t4;
    t126 = t10*t19;
    t129 = t78*t8;
    t139 = -2.0*t57*t34*t18*t23+2.0*t100*t6*t87+2.0*t104*t69-2.0*t100*t18*t108+
4.0*t36*t112+6.0*t43*t35+4.0*t118*t37+t35*t28+2.0*t36*t122+2.0*t125*t126+2.0*
t129*t126+2.0*t37*t122-2.0*t78*t42*t18*t30+t43*t21;
    t141 = t10*t33;
    t144 = t46*t18;
    t150 = t46*t19;
    t153 = t46*t10;
    t161 = t27*t27;
    t167 = 2.0*t36*t141-2.0*t144*t108+2.0*t37*t141+t66*t33+t150*t27+t150*t20+
4.0*t37*t153+6.0*t43*t46+4.0*t112*t10+t43*t33+t161*t42*t19+t43*t26+4.0*t36*t153
;
    t174 = t20*t20;
    t193 = 6.0*t35*t46+4.0*t10*t118+t35*t33+t35*t26+t174*t34*t19+t100*t27*t42+
t100*t20*t34+t100*t19*t20+t90*t46+t65*t161*t42+t90*t26+t49*t27*t42+t49*t20*t34+
t49*t19*t27;
    t199 = t34*t34;
    t201 = t12*t23;
    t202 = t14*t30;
    t213 = t42*t42;
    t219 = t66*t46+t100*t26+t46*t100+t174*t199-2.0*t201*t202-2.0*t144*t51+t46*
t26+t65*t174*t34+t49*t33+t49*t46+t46*t33+t161*t213-2.0*t7*t14*t20*t34;
    t220 = t1*t27;
    t222 = t36*t8;
    t225 = t7*t14;
    t236 = t4*t6*t8;
    t243 = t3*t12;
    t250 = t46*t46;
    t253 = t1*t20;
    t260 = -4.0*t220*t14*t222-4.0*t225*t40-4.0*t220*t15*t10+2.0*t90*t40+2.0*
t225*t24+2.0*t72*t236-2.0*t3*t12*t27*t42-4.0*t243*t44+2.0*t66*t44+2.0*t243*t31+
t250+2.0*t68*t236-4.0*t253*t12*t236-4.0*t253*t13*t10;
    t271 = t4*t20;
    t273 = t8*t18;
    t296 = t10*t18;
    t303 = 2.0*t104*t236-2.0*t35*t31+12.0*t35*t44+2.0*t125*t37*t19-4.0*t271*t6*
t273*t23+2.0*t36*t37*t26+2.0*t36*t129*t19-4.0*t36*t27*t273*t30+2.0*t36*t37*t33+
12.0*t36*t43*t10+12.0*t36*t37*t46-4.0*t271*t296*t23+2.0*t36*t126*t27;
    t317 = t18*t14;
    t331 = t14*t2;
    t335 = t12*t18;
    t339 = t220*t18;
    t342 = t7*t30;
    t345 = t317*t6;
    t350 = -4.0*t31*t40-2.0*t43*t24+2.0*t37*t126*t20-4.0*t44*t24-4.0*t27*t8*
t296*t30-2.0*t253*t317*t30-2.0*t65*t2*t23*t6*t30+2.0*t3*t23*t14*t30-2.0*t12*t19
*t331*t6+2.0*t335*t331*t30-2.0*t201*t339+2.0*t201*t342+2.0*t201*t345+2.0*t86*
t222;
    t354 = 1/(t95+t139+t167+t193+t219+t260+t303+t350);
    t361 = t22*t4+t29*t8+t296-t23*t4-t30*t8-h[5];
    t365 = t253*t18-t3*t23-t335*t2+t201+t339-t342-t345+t202;
    t374 = t66-2.0*t243+t49+t90-2.0*t225+t100+t35+t39+t41+t43+t45+t46;
    err[0] = sqrt((t17*t47*t354-t361*t365*t354)*t17+(-t17*t365*t354+t361*t374*
t354)*t361);
    return;
  }
}

#include <math.h>
void calc2DHomogSampsonErrGrads(double m1[2],double m2[2],double h[9],
double grads[9])
{
  double t1;
  double t10;
  double t100;
  double t1001;
  double t1020;
  double t103;
  double t1030;
  double t106;
  double t1064;
  double t1067;
  double t1071;
  double t108;
  double t1096;
  double t1118;
  double t113;
  double t1137;
  double t114;
  double t1161;
  double t118;
  double t1187;
  double t119;
  double t1190;
  double t12;
  double t123;
  double t126;
  double t129;
  double t13;
  double t130;
  double t133;
  double t134;
  double t139;
  double t14;
  double t141;
  double t149;
  double t15;
  double t150;
  double t153;
  double t158;
  double t159;
  double t161;
  double t162;
  double t167;
  double t17;
  double t173;
  double t176;
  double t18;
  double t180;
  double t19;
  double t191;
  double t2;
  double t20;
  double t204;
  double t21;
  double t210;
  double t213;
  double t215;
  double t219;
  double t22;
  double t224;
  double t23;
  double t230;
  double t231;
  double t234;
  double t238;
  double t24;
  double t241;
  double t242;
  double t257;
  double t259;
  double t26;
  double t261;
  double t262;
  double t263;
  double t266;
  double t269;
  double t27;
  double t272;
  double t276;
  double t277;
  double t28;
  double t280;
  double t283;
  double t287;
  double t29;
  double t290;
  double t299;
  double t3;
  double t30;
  double t302;
  double t303;
  double t306;
  double t307;
  double t31;
  double t310;
  double t313;
  double t314;
  double t318;
  double t319;
  double t322;
  double t323;
  double t329;
  double t33;
  double t332;
  double t333;
  double t338;
  double t34;
  double t343;
  double t346;
  double t35;
  double t350;
  double t353;
  double t354;
  double t357;
  double t36;
  double t361;
  double t365;
  double t366;
  double t368;
  double t37;
  double t370;
  double t374;
  double t375;
  double t377;
  double t380;
  double t381;
  double t384;
  double t385;
  double t39;
  double t4;
  double t40;
  double t402;
  double t405;
  double t408;
  double t41;
  double t42;
  double t422;
  double t423;
  double t428;
  double t429;
  double t43;
  double t436;
  double t438;
  double t44;
  double t440;
  double t448;
  double t45;
  double t452;
  double t46;
  double t47;
  double t48;
  double t484;
  double t49;
  double t50;
  double t501;
  double t51;
  double t512;
  double t514;
  double t516;
  double t524;
  double t528;
  double t54;
  double t55;
  double t559;
  double t566;
  double t567;
  double t58;
  double t589;
  double t596;
  double t598;
  double t6;
  double t61;
  double t62;
  double t63;
  double t634;
  double t643;
  double t66;
  double t672;
  double t674;
  double t69;
  double t695;
  double t696;
  double t698;
  double t7;
  double t70;
  double t703;
  double t712;
  double t72;
  double t722;
  double t741;
  double t742;
  double t745;
  double t748;
  double t75;
  double t756;
  double t76;
  double t764;
  double t777;
  double t784;
  double t79;
  double t792;
  double t8;
  double t80;
  double t81;
  double t813;
  double t822;
  double t84;
  double t851;
  double t864;
  double t867;
  double t869;
  double t87;
  double t870;
  double t874;
  double t88;
  double t880;
  double t886;
  double t895;
  double t899;
  double t901;
  double t902;
  double t909;
  double t91;
  double t922;
  double t94;
  double t943;
  double t946;
  double t95;
  double t96;
  double t983;
  double t99;
  {
    t1 = m2[0];
    t2 = h[6];
    t3 = t2*t1;
    t4 = m1[0];
    t6 = h[7];
    t7 = t1*t6;
    t8 = m1[1];
    t10 = h[8];
    t12 = h[0];
    t13 = t12*t4;
    t14 = h[1];
    t15 = t14*t8;
    t17 = t3*t4+t7*t8+t1*t10-t13-t15-h[2];
    t18 = m2[1];
    t19 = t18*t18;
    t20 = t2*t2;
    t21 = t19*t20;
    t22 = t18*t2;
    t23 = h[3];
    t24 = t23*t22;
    t26 = t23*t23;
    t27 = t6*t6;
    t28 = t19*t27;
    t29 = t18*t6;
    t30 = h[4];
    t31 = t29*t30;
    t33 = t30*t30;
    t34 = t4*t4;
    t35 = t20*t34;
    t36 = t2*t4;
    t37 = t6*t8;
    t39 = 2.0*t36*t37;
    t40 = t36*t10;
    t41 = 2.0*t40;
    t42 = t8*t8;
    t43 = t42*t27;
    t44 = t37*t10;
    t45 = 2.0*t44;
    t46 = t10*t10;
    t47 = t21-2.0*t24+t26+t28-2.0*t31+t33+t35+t39+t41+t43+t45+t46;
    t48 = t17*t47;
    t49 = t12*t12;
    t50 = t2*t49;
    t51 = t4*t10;
    t54 = t49*t18;
    t55 = t6*t30;
    t58 = t12*t46;
    t61 = t20*t2;
    t62 = t1*t61;
    t63 = t12*t34;
    t66 = t12*t33;
    t69 = t1*t1;
    t70 = t69*t20;
    t72 = t69*t61;
    t75 = t61*t34;
    t76 = t18*t23;
    t79 = t14*t14;
    t80 = t79*t6;
    t81 = t8*t10;
    t84 = t79*t2;
    t87 = t79*t18;
    t88 = t2*t23;
    t91 = t14*t46;
    t94 = t27*t6;
    t95 = t1*t94;
    t96 = t14*t42;
    t99 = 2.0*t50*t51-2.0*t54*t55-2.0*t3*t58-2.0*t62*t63-2.0*t3*t66+t70*t43+2.0
*t72*t51-2.0*t75*t76+2.0*t80*t81+2.0*t84*t51-2.0*t87*t88-2.0*t7*t91-2.0*t95*t96
;
    t100 = t14*t26;
    t103 = t69*t94;
    t106 = t69*t27;
    t108 = t49*t6;
    t113 = t34*t4;
    t114 = t61*t113;
    t118 = t94*t42;
    t119 = t18*t30;
    t123 = t10*t33;
    t126 = t10*t26;
    t129 = t61*t4;
    t130 = t10*t19;
    t133 = t42*t8;
    t134 = t94*t133;
    t139 = -2.0*t7*t100+2.0*t103*t81+t106*t35+2.0*t108*t81+6.0*t43*t35+4.0*t114
*t37+t35*t28-2.0*t118*t119+t43*t21+2.0*t36*t123+2.0*t36*t126+2.0*t129*t130+4.0*
t36*t134+2.0*t37*t123;
    t141 = t94*t8;
    t149 = t12*t23;
    t150 = t14*t30;
    t153 = t46*t18;
    t158 = t20*t20;
    t159 = t34*t34;
    t161 = t27*t27;
    t162 = t42*t42;
    t167 = 2.0*t141*t130+2.0*t37*t126+t79*t26+t46*t26+t46*t33-2.0*t149*t150-2.0
*t153*t55-2.0*t153*t88+t158*t159+t162*t161+t79*t46+t49*t46+t49*t33;
    t173 = t46*t10;
    t176 = t46*t19;
    t180 = t46*t46;
    t191 = t43*t33+4.0*t134*t10+6.0*t43*t46+4.0*t37*t173+t176*t20+t176*t27+t70*
t33+t180+t79*t20*t34+t79*t27*t42+t158*t34*t19+t35*t26+t35*t33+4.0*t114*t10;
    t204 = t49*t19;
    t210 = t7*t14;
    t213 = t1*t27;
    t215 = t36*t8;
    t219 = t14*t20*t34;
    t224 = 6.0*t35*t46+4.0*t36*t173+t43*t26+t161*t42*t19+t69*t158*t34+t70*t46+
t204*t27+t49*t20*t34+t49*t27*t42-4.0*t210*t40-4.0*t213*t14*t215-2.0*t7*t219+2.0
*t210*t24;
    t230 = t4*t6;
    t231 = t230*t8;
    t234 = t3*t12;
    t238 = t12*t27*t42;
    t241 = t1*t20;
    t242 = t13*t10;
    t257 = t79*t19;
    t259 = t106*t26+2.0*t106*t40+2.0*t103*t215+2.0*t50*t231-4.0*t234*t44-2.0*t3
*t238-4.0*t242*t241-4.0*t241*t12*t231+2.0*t31*t234+t69*t161*t42+2.0*t70*t44+2.0
*t72*t231+t106*t46+t257*t20;
    t261 = t4*t20;
    t262 = t10*t18;
    t263 = t262*t23;
    t266 = t46*t37;
    t269 = t43*t10;
    t272 = t33*t37;
    t276 = t8*t18;
    t277 = t276*t30;
    t280 = t141*t19;
    t283 = t37*t26;
    t287 = t276*t23;
    t290 = t37*t19;
    t299 = t15*t10;
    t302 = -4.0*t261*t263+12.0*t36*t266+12.0*t36*t269+2.0*t36*t272-4.0*t36*t27*
t277+2.0*t36*t280+2.0*t36*t283-4.0*t261*t6*t287+2.0*t129*t290+12.0*t35*t44-2.0*
t35*t31+2.0*t84*t231-4.0*t213*t299;
    t303 = t7*t30;
    t306 = t18*t14;
    t307 = t306*t6;
    t310 = t213*t18;
    t313 = t12*t18;
    t314 = t14*t2;
    t318 = t23*t14;
    t319 = t318*t30;
    t322 = t69*t2;
    t323 = t23*t6;
    t329 = t306*t30;
    t332 = t27*t8;
    t333 = t262*t30;
    t338 = t130*t20;
    t343 = t130*t27;
    t346 = t12*t19;
    t350 = 2.0*t303*t149+2.0*t149*t307-2.0*t310*t149+2.0*t313*t314*t30+2.0*t3*
t319-2.0*t322*t323*t30-4.0*t44*t24-2.0*t241*t329-4.0*t332*t333-4.0*t31*t40+2.0*
t37*t338-2.0*t43*t24+2.0*t36*t343-2.0*t346*t314*t6;
    t353 = t99+t139+t167+t191+t224+t259+t302+t350;
    t354 = 1/t353;
    t357 = t29*t8;
    t361 = t22*t4+t357+t262-t23*t4-t30*t8-h[5];
    t365 = t241*t18-t3*t23-t313*t2+t149+t310-t303-t307+t150;
    t366 = t361*t365;
    t368 = t48*t354-t366*t354;
    t370 = t17*t365;
    t374 = t70-2.0*t234+t49+t106-2.0*t210+t79+t35+t39+t41+t43+t45+t46;
    t375 = t361*t374;
    t377 = -t370*t354+t375*t354;
    t380 = sqrt(t368*t17+t377*t361);
    t381 = 1/t380;
    t384 = t353*t353;
    t385 = 1/t384;
    t402 = t12*t2;
    t405 = t12*t6;
    t408 = 2.0*t238+2.0*t12*t20*t34+2.0*t346*t27+2.0*t66+2.0*t58-2.0*t3*t33-2.0
*t62*t34-2.0*t3*t46-4.0*t313*t55+4.0*t402*t51+4.0*t405*t81;
    t422 = t19*t2;
    t423 = t14*t6;
    t428 = t23*t1;
    t429 = t27*t18;
    t436 = -2.0*t319+2.0*t3*t31-4.0*t241*t231-4.0*t241*t51-2.0*t3*t43-4.0*t3*
t44+4.0*t402*t231-2.0*t422*t423+2.0*t22*t150-2.0*t428*t429+2.0*t428*t55+2.0*
t318*t29;
    t438 = t385*(t408+t436);
    t440 = -t22+t23;
    t448 = t4*t365*t354;
    t452 = -t3+t12;
    grads[0] = t381*((-t4*t47*t354-t48*t438-t361*t440*t354+t366*t438)*t17-t368*
t4+(t448-t17*t440*t354+t370*t438+2.0*t361*t452*t354-t375*t438)*t361)/2.0;
    t484 = 2.0*t14*t27*t42+2.0*t219+2.0*t14*t19*t20+2.0*t100+2.0*t91-2.0*t7*t26
-2.0*t95*t42-2.0*t7*t46-4.0*t306*t88+4.0*t51*t314+4.0*t423*t81;
    t501 = t23*t30;
    t512 = -2.0*t149*t30+2.0*t7*t24-2.0*t7*t35-4.0*t215*t213-4.0*t7*t40-4.0*
t213*t81+4.0*t314*t231-2.0*t241*t119+2.0*t3*t501-2.0*t346*t2*t6+2.0*t313*t2*t30
+2.0*t149*t29;
    t514 = t385*(t484+t512);
    t516 = -t29+t30;
    t524 = t8*t365*t354;
    t528 = -t7+t14;
    grads[1] = t381*((-t8*t47*t354-t48*t514-t361*t516*t354+t366*t514)*t17-t368*
t8+(t524-t17*t516*t354+t370*t514+2.0*t361*t528*t354-t375*t514)*t361)/2.0;
    grads[2] = -t381*t368;
    t559 = t10*t23;
    t566 = 2.0*t43*t23+2.0*t35*t23+2.0*t106*t23+2.0*t79*t23+2.0*t46*t23-4.0*
t318*t7-2.0*t87*t2-2.0*t75*t18+4.0*t36*t559+4.0*t37*t559-2.0*t153*t2;
    t567 = t12*t14;
    t589 = t1*t12;
    t596 = -2.0*t567*t30+2.0*t7*t306*t2-4.0*t261*t357+4.0*t36*t37*t23-4.0*t261*
t262-2.0*t43*t22-4.0*t37*t262*t2-2.0*t322*t55+2.0*t3*t150-2.0*t589*t429+2.0*
t589*t55+2.0*t567*t29;
    t598 = t385*(t566+t596);
    grads[3] = t381*((2.0*t17*t440*t354-t48*t598+t448-t361*t452*t354+t598*t366)
*t17+(-t17*t452*t354+t370*t598-t4*t374*t354-t375*t598)*t361-t377*t4)/2.0;
    t634 = t10*t30;
    t643 = 2.0*t70*t30+2.0*t43*t30+2.0*t30*t35+2.0*t49*t30+2.0*t46*t30-4.0*t3*
t12*t30-2.0*t54*t6+4.0*t36*t634-2.0*t118*t18+4.0*t37*t634-2.0*t153*t6;
    t672 = -2.0*t149*t14+2.0*t3*t313*t6-2.0*t35*t29-4.0*t36*t332*t18+4.0*t36*
t37*t30-4.0*t36*t262*t6-4.0*t332*t262-2.0*t241*t306-2.0*t322*t323+2.0*t3*t318+
2.0*t313*t314+2.0*t149*t7;
    t674 = t385*(t643+t672);
    grads[4] = t381*((2.0*t17*t516*t354-t48*t674+t524-t361*t528*t354+t366*t674)
*t17+(-t17*t528*t354+t370*t674-t8*t374*t354-t375*t674)*t361-t377*t8)/2.0;
    grads[5] = -t381*t377;
    t695 = t1*t4;
    t696 = t47*t354;
    t698 = t2*t34;
    t703 = t4*t27;
    t712 = t36*t6;
    t722 = t20*t113;
    t741 = -4.0*t703*t277-8.0*t36*t263+24.0*t698*t44-4.0*t698*t31-8.0*t712*t287
-2.0*t153*t23-8.0*t3*t242+2.0*t7*t306*t23+12.0*t722*t10+4.0*t72*t34+2.0*t322*
t46+2.0*t50*t34+2.0*t257*t2+2.0*t84*t34+4.0*t75*t19-4.0*t589*t44+2.0*t698*t26;
    t742 = t79*t4;
    t745 = t8*t26;
    t748 = t8*t46;
    t756 = t14*t4;
    t764 = t49*t4;
    t777 = t4*t94;
    t784 = 2.0*t742*t37+2.0*t230*t745+12.0*t230*t748+12.0*t703*t42*t10+2.0*t698
*t33-4.0*t7*t756*t10+12.0*t698*t46-2.0*t87*t23+2.0*t764*t10-2.0*t589*t46-2.0*
t589*t33+2.0*t51*t33+2.0*t51*t26-4.0*t3*t329+4.0*t777*t133+2.0*t742*t10+6.0*
t261*t290;
    t792 = t8*t19;
    t813 = t4*t8;
    t822 = 2.0*t176*t2+2.0*t322*t33+2.0*t51*t28+2.0*t777*t792+6.0*t261*t130+
12.0*t698*t43+12.0*t722*t37+2.0*t698*t28-6.0*t35*t76-8.0*t234*t231+2.0*t43*t422
+2.0*t589*t31+2.0*t106*t51+2.0*t103*t813+2.0*t764*t37+4.0*t322*t44-2.0*t589*t43
;
    t851 = t8*t33;
    t864 = 6.0*t70*t231-6.0*t241*t63+2.0*t106*t698+2.0*t322*t43+4.0*t4*t173+4.0
*t61*t159-4.0*t7*t314*t34-4.0*t213*t756*t8+6.0*t70*t51+2.0*t428*t150-2.0*t346*
t423+2.0*t313*t150-2.0*t76*t43+2.0*t230*t851-2.0*t69*t23*t55-4.0*t51*t31+4.0*
t37*t130*t2-4.0*t37*t263;
    t867 = t385*(t741+t784+t822+t864);
    t869 = t18*t4;
    t870 = t365*t354;
    t874 = 2.0*t3*t18-t428-t313;
    t880 = t368*t1;
    t886 = t374*t354;
    t895 = t377*t18;
    grads[6] = t381*((t695*t696+t17*(2.0*t422-2.0*t76+2.0*t698+2.0*t231+2.0*t51
)*t354-t48*t867-t869*t870-t361*t874*t354+t366*t867)*t17+t880*t4+(-t695*t870-t17
*t874*t354+t370*t867+t869*t886+t361*(2.0*t322-2.0*t589+2.0*t698+2.0*t231+2.0*
t51)*t354-t375*t867)*t361+t895*t4)/2.0;
    t899 = t1*t8;
    t901 = t19*t6;
    t902 = t6*t42;
    t909 = t1*t14;
    t922 = t27*t133;
    t943 = t69*t6;
    t946 = 2.0*t72*t813-2.0*t909*t35+2.0*t84*t813-6.0*t43*t119+6.0*t332*t130+
2.0*t35*t901+12.0*t35*t902+12.0*t922*t36+2.0*t902*t21-4.0*t241*t13*t8-4.0*t3*
t405*t42+2.0*t909*t24-8.0*t210*t215+12.0*t902*t46+2.0*t902*t26+2.0*t108*t42+2.0
*t943*t26;
    t983 = 4.0*t103*t42+2.0*t943*t46+2.0*t80*t42-2.0*t54*t30+2.0*t49*t8*t10-2.0
*t46*t909-2.0*t909*t26+4.0*t114*t8+2.0*t79*t8*t10-2.0*t153*t30+2.0*t81*t33-4.0*
t909*t40-8.0*t7*t299+2.0*t176*t6+2.0*t204*t6+4.0*t118*t19+2.0*t902*t33;
    t1001 = t1*t30;
    t1020 = 12.0*t922*t10+6.0*t106*t81-6.0*t213*t96+2.0*t70*t902+2.0*t943*t35+
2.0*t149*t306-2.0*t501*t322-2.0*t346*t314+2.0*t149*t1001+2.0*t36*t851+12.0*t36*
t748+2.0*t81*t21-2.0*t35*t119+12.0*t35*t81+2.0*t129*t792+2.0*t36*t745+4.0*t94*
t162;
    t1030 = t7*t18;
    t1064 = -8.0*t37*t333-4.0*t902*t24-8.0*t712*t277+24.0*t36*t902*t10-4.0*t149
*t1030-4.0*t81*t24+4.0*t36*t130*t6-4.0*t36*t333+6.0*t36*t332*t19-4.0*t261*t287+
2.0*t50*t813+2.0*t70*t81+4.0*t8*t173-4.0*t3*t12*t8*t10+2.0*t81*t26+6.0*t106*
t215+4.0*t943*t40+2.0*t3*t313*t30;
    t1067 = t385*(t946+t983+t1020+t1064);
    t1071 = 2.0*t1030-t1001-t306;
    grads[7] = t381*((t899*t696+t17*(2.0*t901-2.0*t119+2.0*t215+2.0*t902+2.0*
t81)*t354-t48*t1067-t276*t870-t361*t1071*t354+t366*t1067)*t17+t880*t8+(-t899*
t870-t17*t1071*t354+t370*t1067+t276*t886+t361*(2.0*t943-2.0*t909+2.0*t215+2.0*
t902+2.0*t81)*t354-t375*t1067)*t361+t895*t8)/2.0;
    t1096 = 2.0*t36+2.0*t37+2.0*t10;
    t1118 = 2.0*t108*t8+2.0*t50*t4+2.0*t72*t4+2.0*t272+12.0*t35*t10+12.0*t36*
t46+2.0*t280+2.0*t283+2.0*t36*t33+2.0*t36*t26+2.0*t129*t19;
    t1137 = 2.0*t80*t8+2.0*t84*t4+2.0*t103*t8+2.0*t343+12.0*t266+2.0*t338+2.0*
t70*t10+2.0*t106*t10+12.0*t269+4.0*t173+2.0*t126+2.0*t79*t10;
    t1161 = 4.0*t134-4.0*t3*t405*t8-4.0*t7*t314*t4+24.0*t36*t44-4.0*t37*t24-4.0
*t36*t31+2.0*t123+2.0*t49*t10+2.0*t70*t37+12.0*t36*t43+12.0*t35*t37;
    t1187 = -4.0*t213*t15+2.0*t106*t36-4.0*t332*t119+2.0*t37*t21+4.0*t114+2.0*
t36*t28-4.0*t261*t76-4.0*t241*t13-4.0*t7*t14*t10-4.0*t262*t55-4.0*t262*t88-4.0*
t3*t12*t10;
    t1190 = t385*(t1118+t1137+t1161+t1187);
    grads[8] = t381*((t1*t47*t354+t17*t1096*t354-t48*t1190-t18*t365*t354+t366*
t1190)*t17+t880+(-t1*t365*t354+t370*t1190+t18*t374*t354+t361*t1096*t354-t375*
t1190)*t361+t895)/2.0;
    return;
  }
}

#include <math.h>
void calc2DHomogReprojection(double h[9],double pm1[2],double pm12[4])
{
  double t13;
  {
    pm12[0] = pm1[0];
    pm12[1] = pm1[1];
    t13 = 1/(h[6]*pm12[0]+h[7]*pm12[1]+h[8]);
    pm12[2] = (h[0]*pm12[0]+h[1]*pm12[1]+h[2])*t13;
    pm12[3] = (h[3]*pm12[0]+h[4]*pm12[1]+h[5])*t13;
    return;
  }
}

void calc2DHomogReprojectionJac(double h[9],double pm1[2],double hgrad2[9],
double hgrad3[9],double pm1grad0[2],double pm1grad1[2],double pm1grad2[2],
double pm1grad3[2])
{
  double t1;
  double t10;
  double t12;
  double t16;
  double t17;
  double t18;
  double t2;
  double t21;
  double t23;
  double t27;
  double t4;
  double t5;
  double t8;
  double t9;
  {
    t1 = pm1[0];
    t2 = h[6];
    t4 = h[7];
    t5 = pm1[1];
    t8 = t2*t1+t5*t4+h[8];
    t9 = 1/t8;
    hgrad2[0] = t1*t9;
    hgrad2[1] = t5*t9;
    hgrad2[2] = t9;
    hgrad2[3] = 0.0;
    hgrad2[4] = 0.0;
    hgrad2[5] = 0.0;
    t10 = h[0];
    t12 = h[1];
    t16 = t8*t8;
    t17 = 1/t16;
    t18 = (t1*t10+t12*t5+h[2])*t17;
    hgrad2[6] = -t18*t1;
    hgrad2[7] = -t18*t5;
    hgrad2[8] = -t18;
    hgrad3[0] = 0.0;
    hgrad3[1] = 0.0;
    hgrad3[2] = 0.0;
    hgrad3[3] = hgrad2[0];
    hgrad3[4] = hgrad2[1];
    hgrad3[5] = hgrad2[2];
    t21 = h[3];
    t23 = h[4];
    t27 = (t21*t1+t5*t23+h[5])*t17;
    hgrad3[6] = -t1*t27;
    hgrad3[7] = -t27*t5;
    hgrad3[8] = -t27;
    pm1grad0[0] = 1.0;
    pm1grad0[1] = 0.0;
    pm1grad1[0] = 0.0;
    pm1grad1[1] = 1.0;
    pm1grad2[0] = t10*hgrad3[5]-t18*t2;
    pm1grad2[1] = t12*hgrad3[5]-t18*t4;
    pm1grad3[0] = t21*hgrad3[5]-t27*t2;
    pm1grad3[1] = t23*hgrad3[5]-t4*t27;
    return;
  }
}

