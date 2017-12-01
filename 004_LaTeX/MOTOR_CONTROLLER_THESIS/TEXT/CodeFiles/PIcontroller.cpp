	error = reference_value - measured_value;
	output = (error * kp) + integral_error;
    if (output > MAX_VALUE)
    	output = MAX_VALUE;
    else if (u < MIN_VALUE)
    	output = MIN_VALUE;
    else
    	integral_error = integral_error + ((ki * error) * dt);