import numpy as np

def applyLowPass(buffers, value):
    """
    Description
    -----------
    Apply a 4th order low pass IIR filter.
    The filter cutoff frequency is 2 Hz.
    The difference equation requires the past 3 previous input/output values.
    
    Parameters
    ----------
    buffers: list
        A list of 6 previous sensor readings (3 input + 3 output)
        Used in the filter equation to compute the output at time k.
    value : float
        Accelerometer or Gyroscope reading obtained from the sensor.

    Returns
    -------
    buffers: list
        Update list of the previous input/output values.
    value : float
        Returns the calculated filter output.
    
    """
    in_buffer = buffers[0:3]
    out_buffer = buffers[3:6]


    # Filter coefficients.
    b_coef = [ 0.00420469, -0.00371297, -0.00371297,  0.00420469]
    a_coef = [ 1., -2.86540099,  2.75073423, -0.8843498 ]
    
    # Compute filter output at time k based on the difference equation.
    filter_out = b_coef[0] * value + b_coef[1] * in_buffer[2] +\
                    b_coef[2] * in_buffer[1] + b_coef[3] * in_buffer[0] -\
                    a_coef[1] * out_buffer[2] - a_coef[2] * out_buffer[1] -\
                    a_coef[3] * out_buffer[0]
    
    # Update the list of previous input, output values.
    out_buffer = np.append(out_buffer, filter_out)
    out_buffer = out_buffer[-3:]

    in_buffer = np.append(in_buffer, value)
    in_buffer = in_buffer[-3:]

    return np.concatenate((in_buffer, out_buffer)),  filter_out