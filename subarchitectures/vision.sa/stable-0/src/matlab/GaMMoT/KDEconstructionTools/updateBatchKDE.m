function [kernel_pdf, C_curr] = updateBatchKDE( kernel_pdf, data, varargin )
% 'bwmethod' = 'Silverman' , 'Plugin'
kernel_pdf = estimateKDEfromData( data, varargin{:} ) ;
 