function output_kde_cl = executeOperatorIKDEClsfr( input_kde_cl, varargin )
 
operator_data = [] ;
input_data = [] ;
vforwvargin = {} ;
args = varargin;
nargs = length(args);
i = 1 ;
while i <= nargs   
    switch args{i}        
        case 'input_data', input_data = args{i+1} ; i = i + 2 ; 
        case 'add_input', operator_data = args{i} ; i = i + 1 ; 
        case 'compress_pdf', operator_data = args{i} ; i = i + 1 ;
        case 'classifyData', operator_data = args{i} ; i = i + 1 ; 
        otherwise
             vforwvargin = horzcat(vforwvargin, args{i} ) ;
             i = i + 1 ; 
    end
end

switch operator_data
    case 'add_input'
        % initialize if empty input_kde
        if isempty(input_kde_cl)
            output_kde_cl = {} ;
            for i = 1 : length(input_data)
                %         class = input_data{i}.class ;
                kde = executeOperatorIKDE( [], 'input_data', input_data{i}.data, 'add_input', vforwvargin{:}  ) ;
                output_kde_cl = horzcat(output_kde_cl, kde) ;
            end
            return ;
        else
            % store previous
            output_kde_cl = input_kde_cl ;
            % add input to kdes
            for i = 1 : length(input_data)
                class = input_data{i}.class ;
                data = input_data{i}.data ;
                
                if isempty(data)
                    continue ; 
                end
                % make negative data model
                otherClasses = makeOtherClasses( input_kde_cl, class ) ; %input_kde_cl  output_kde_cl
                % update the kde
                kde = executeOperatorIKDE( input_kde_cl{class}, 'input_data', ...
                    data, 'add_input',...
                    'otherClasses', otherClasses, vforwvargin{:} ) ;
                output_kde_cl{class} = kde ;
            end
        end
    case 'compress_pdf'
        % store previous
        output_kde_cl = input_kde_cl ;
        for class = 1 : length(input_kde_cl)
            % make negative data model
            otherClasses = makeOtherClasses( output_kde_cl, class ) ; %input_kde_cl
            % update the kde
            kde = executeOperatorIKDE( input_kde_cl{class}, 'compress_pdf',...
                                       'otherClasses', otherClasses, vforwvargin{:} ) ;
            output_kde_cl{class} = kde ;
        end
    case 'classifyData'
        P = zeros(length(input_kde_cl),size(input_data,2)) ;
        for i = 1 : length(input_kde_cl)
            p = evaluatePointsUnderPdf(input_kde_cl{i}.pdf, input_data) ;
            P(i,:) = p ;
        end
        output_kde_cl = [] ;
        output_kde_cl.P = P ;
        [vals, output_kde_cl.C] = max(P) ;
end

% -------------------------------------------------------------------- %
function otherClasses = makeOtherClasses( input_kde_cl, i_exclude )

otherClasses.pdfs = {} ; 
w_other = ones(1,length(input_kde_cl)-1) ;  
w_other = w_other / sum(w_other) ;
otherClasses.priors = (length(input_kde_cl)-1) / length(input_kde_cl) ;  
otherClasses.inner_priors = w_other ;
for i = 1 : length(input_kde_cl) 
    if i ~= i_exclude
        otherClasses.pdfs = horzcat(otherClasses.pdfs, input_kde_cl{i}.pdf) ; 
    end
end




 