function observation_probability = observationModel(map_, row_, col_, observations_)
  observation_probability = 1;

  #evaluate cell occupancy
  cell_up_occupied    = 0;
  cell_down_occupied  = 0;
  cell_left_occupied  = 0;
  cell_right_occupied = 0;
  if (row_-1 > 0)
	  cell_up_occupied    = map_(row_-1, col_);
  endif
  if (row_+1 <= rows(map_))
    cell_down_occupied  = map_(row_+1, col_);
  endif
  if (col_-1 > 0)
	  cell_left_occupied  = map_(row_, col_-1);
  endif
  if (col_+1 <= columns(map_))
	  cell_right_occupied = map_(row_, col_+1);
  endif

  #update probability depending on observations
  if (cell_up_occupied == observations_(1))
	  observation_probability *= 0.8; #true positive
  else
	  observation_probability *= 0.2; #false positive
  endif	    
  if (cell_down_occupied == observations_(2))
	  observation_probability *= 0.8; #true positive
  else
	  observation_probability *= 0.2; #false positive
  endif
  if (cell_left_occupied == observations_(3))
	  observation_probability *= 0.8; #true positive
  else
	  observation_probability *= 0.2; #false positive
  endif	
  if (cell_right_occupied == observations_(4))
	  observation_probability *= 0.8; #true positive
  else
	  observation_probability *= 0.2; #false positive
  endif	
endfunction

