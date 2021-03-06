clc
clear
close all
% %% Y P R 
% %???姿�????
% r=0;
% p=-90;
% y=-45;
% ankle_from_ground = 3;
% %???Z_a�?aw�?
% Y = rotz(y);
% %???Y_b�?itch�?
% P = roty(p);
% %???�?_b�?oll�?
% R = rotx(r);
% %??????顺�??��???��????��?,?��?�??�????
% T_r = Y*P*R;
% %????��???????�?�?????
% P_b = [0 0 ankle_from_ground]';
% %????��????�?????�?�?????
% P_a = T_r*P_b
% %�??�??,以�??��?�??正确,??���??????��????�?????�?�?????
% 
% %%
% %?��??��??��?�???��?�??�?????,rotx/y/z?��?�??????��?�?% syms yy pp rr ankle_from_ground
% % Y_symbol=[  cos(yy)     -sin(yy)    0;
% %             sin(yy)     cos(yy)     0;
% %             0           0           1];
% % 
% % P_symbol=[  cos(pp)     0           sin(pp);
% %             0           1           0      ;
% %             -sin(pp)    0           cos(pp)];
% % 
% % R_symbol=[  1           0           0       ;
% %             0           cos(rr)     -sin(rr);
% %             0           sin(rr)     cos(rr) ];
% %         
% %         
% % T_r_symbol = Y_symbol * P_symbol * R_symbol;
% % P_b_symbol = [0; 0; ankle_from_ground];
% % P_a_symbol = T_r_symbol*P_b_symbol
% % % �?��P_a_symbol�????��?��?C++代�?�??�??�?% 
% 
% %%
% %�?????�??�???��?�?��?��????�?��??
% %???ankle_roll??��线�???????�????p=-45;
% y=45;
% %???Z_a�?aw�?Y = rotz(y);
% %???Y_b�?itch�?P = roty(p);
% %??????顺�??��???��????��?,?��?�??�????T_r = Y*P;
% %�?nkle_roll??��轴�?B???系中???��????�????P_b = [1 0 0]';
% %?��?ankle_roll�?��???对�??????P_a = T_r*P_b
% 
% %%
% %�??ankle_roll轴线?��????????�表达�?
% syms yy pp rr ankle_from_ground fpx fpy fpz hfox hfoy hfoz
% Y_symbol=[  cos(yy)     -sin(yy)    0;
%             sin(yy)     cos(yy)     0;
%             0           0           1];
% 
% P_symbol=[  cos(pp)     0           sin(pp);
%             0           1           0      ;
%             -sin(pp)    0           cos(pp)];
%         
% R_symbol=[  1           0           0       ;
%             0           cos(rr)     -sin(rr);
%             0           sin(rr)     cos(rr) ];
% 
% T_r_ankle_axis = Y_symbol * P_symbol;
% P_b_ankle_axis = [1; 0; 0];
% P_a_ankle_axis = T_r_ankle_axis*P_b_ankle_axis
% 
% T_r_symbol = Y_symbol * P_symbol * R_symbol;
% P_b_symbol = [0; 0; ankle_from_ground];
% P_a_symbol =[fpx;fpy;fpz]+ T_r_symbol*P_b_symbol-[hfox;hfoy;hfoz]
% 
% 
% %%
% %?��????hip_roll??ip_yaw�?????hip_pitch???
% syms a b c
% Y_symbol=[  cos(yy)     -sin(yy)    0;
%             sin(yy)     cos(yy)     0;
%             0           0           1];
%         
% R_symbol=[  1           0           0       ;
%             0           cos(rr)     -sin(rr);
%             0           sin(rr)     cos(rr) ];
% 
% T_r_symbol = Y_symbol * R_symbol;
% P_a_symbol = [a; b; c];
% P_b_symbol = simplify(T_r_symbol\P_a_symbol)

%%
%这里测试脚踝的结算
% syms yyfuck ppfuck rrfuck afuck bfuck cfuck
% Y_symbol=[  cos(yyfuck)     -sin(yyfuck)    0;
%             sin(yyfuck)     cos(yyfuck)     0;
%             0           0           1];
% 
% P_symbol=[  cos(ppfuck)     0           sin(ppfuck);
%             0           1           0      ;
%             -sin(ppfuck)    0           cos(ppfuck)];
%         
% R_symbol=[  1           0           0       ;
%             0           cos(rrfuck)     -sin(rrfuck);
%             0           sin(rrfuck)     cos(rrfuck) ];
% 
% 
% T_r_symbol = Y_symbol*R_symbol*P_symbol;
% P_a_symbol = [afuck; bfuck; cfuck];
% P_b_symbol = simplify(T_r_symbol\P_a_symbol)

%%
% P_b_symbol = [0;0;-1];
% P_a_symbol = T_r_symbol*P_b_symbol

%%
% 算全身逆运动学
syms xm ym zm 
syms rq pq yq  d a 
Yq=       [  cos(yq)     -sin(yq)    0;
            sin(yq)     cos(yq)     0;
            0           0           1];

Pq=       [  cos(pq)     0           sin(pq);
            0           1           0      ;
            -sin(pq)    0           cos(pq)];

Rq=       [  1           0           0       ;
            0           cos(rq)     -sin(rq);
            0           sin(rq)     cos(rq) ];

        
A = [Yq,[0;0;0];
    [0,0,0,1]]*[1,0,0,0;
 0,1,0,0;
 0,0,1,d;
 0,0,0,1]*[1,0,0,a;
 0,1,0,0;
 0,0,1,0;
 0,0,0,1]*[Rq,[0;0;0];[0,0,0,1]]

A = subs(A,yq,1.57)
A = subs(A,rq,2)
A = subs(A,a,4.5)
A = subs(A,d,0)
double(A(2,4))

STD = simplify(Yq*Pq*Rq)


DD = (Yq*Rq)\[xm;ym;zm]
DD = simplify(DD)