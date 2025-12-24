function [x,t,save_state,sum_k,Num] = RK4(x,deriv,save_state,sum_k,t,Num,dt)

switch (Num)
    case 1
        save_state = x;
        sum_k = deriv;
        x = save_state + deriv / 2 * dt;
        t = t + dt/2;
        Num = Num +1;
    case 2
        sum_k = sum_k + deriv*2;
        x = save_state + deriv / 2 * dt;
        Num = Num +1;
    case 3
        sum_k = sum_k + deriv*2;
        x = save_state + deriv * dt;
        t = t + dt/2;
        Num = Num +1;
    case 4
        sum_k = sum_k + deriv;
        x = save_state + dt/6 *sum_k;
        Num = Num +1;
    otherwise
        disp('d')
end
