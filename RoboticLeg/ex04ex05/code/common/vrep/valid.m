rBF_valid = double(subs(r_BF_inB,[alpha beta gamma],qGoal'));
error_valid = norm(rBF_valid-rGoal)
if(error_valid < 1E-6)
    disp('CORRECT');
else
    disp('INCORRECT');
end