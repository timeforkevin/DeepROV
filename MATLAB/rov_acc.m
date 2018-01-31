function [ acc ] = rov_acc( x, u, rov )
%rov_acc Calculates ROV accelerations from state and inputs
%   Detailed explanation goes here
    acc = [(u(1)+u(2) - abs(x(7))*x(7)*rov.B(1))/rov.M;
           0
           (u(3)+u(4)+u(5) - abs(x(9))*x(9)*rov.B(3))/rov.M;
           (rov.W(2)*(u(5)-u(3)) - abs(x(10))*x(10)*rov.B(4))/rov.J(1);
           (rov.L(2)*u(4)-rov.L(1)*(u(3)+u(5)) - abs(x(11))*x(11)*rov.B(5))/rov.J(2); %neglect off axis effect on bottom
           (rov.W(1)*(u(2)-u(1)) - abs(x(12))*x(12)*rov.B(6))/rov.J(3);]; %neglect off axis effects for sides

end

