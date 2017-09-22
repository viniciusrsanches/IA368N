load ca


if(isequal(double(r_BF_inBx1), double(subs(r_BF_inB, [alpha beta gamma], double(q1)))) &&...
        isequal(double(r_BF_inBx2), double(subs(r_BF_inB, [alpha beta gamma], double(q2)))) &&...
        isequal(double(r_BF_inBx3), double(subs(r_BF_inB, [alpha beta gamma], double(q3)))))
    disp('CORRECT');
else
    disp('INCORRECT');
end
