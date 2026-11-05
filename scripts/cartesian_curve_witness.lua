-- Pure bounded search for an EXPLICIT point on an authorized joint curve.
-- Finding a witness is not a proof of global minimization/interpolation.
-- At most three closest boxes, three seeds, three Newton updates; no search
-- retry or tolerance relaxation. No actuator, clocks, FK or admission here.
local M={}
function M.closest(curves,target,evaluate,cooperate)
    assert(type(curves)=='table' and #curves>=1 and #curves<=8,'curve_witness_curves')
    assert(type(target)=='table' and #target==6 and type(evaluate)=='function','curve_witness_target')
    for i=1,6 do assert(type(target[i])=='number' and target[i]==target[i]
        and math.abs(target[i])<1000000,'curve_witness_target') end
    local best=nil;local best_cost=math.huge;local ordered={};local evaluated=0
    for _,curve in ipairs(curves) do
        local lower=0
        for i=1,6 do
            local d=math.max(curve[i][1]-target[i],target[i]-curve[i][2],0)
            lower=lower+d*d
        end
        ordered[#ordered+1]={curve=curve,lower=lower}
    end
    table.sort(ordered,function(a,b) return a.lower<b.lower end)
    for _,item in ipairs(ordered) do
        local curve=item.curve
        if evaluated>=3 then break end
        if item.lower<=best_cost then
            evaluated=evaluated+1
            -- Build the scalar squared-distance polynomial ONCE. Newton's
            -- gradient/Hessian then need scalar Horner passes, not a complete
            -- 18-component P/V/A evaluation at every search iteration.
            local cost_poly={};for i=1,11 do cost_poly[i]=0 end
            for axis=1,6 do
                local row=curve.coefficients[axis];local r={row[1]-target[axis],row[2],row[3],row[4],row[5],row[6]}
                for i=0,5 do for j=0,5 do
                    cost_poly[i+j+1]=cost_poly[i+j+1]+r[i+1]*r[j+1]
                end end
            end
            local gradient,hessian={},{}
            for degree=1,10 do gradient[degree]=degree*cost_poly[degree+1] end
            for degree=2,10 do hessian[degree-1]=degree*(degree-1)*cost_poly[degree+1] end
            local function scalar(poly,u)
                local value=poly[#poly]
                for i=#poly-1,1,-1 do value=value*u+poly[i] end
                return value
            end
            local function consider(u)
                -- Direct residuals avoid subtracting nearly equal scalar
                -- cost-polynomial values when deciding which witness to keep.
                local cost=0
                for axis=1,6 do
                    local row=curve.coefficients[axis];local q=row[6]
                    for degree=4,0,-1 do q=q*u+row[degree+1] end
                    local r=q-target[axis];cost=cost+r*r
                end
                if cost<best_cost then best_cost=cost;best={curve=curve,u=u,cost=cost} end
            end
            consider(0);consider(1)
            for seed=0,2 do
                local u=seed/2
                for iteration=1,3 do
                    local g,h=scalar(gradient,u),scalar(hessian,u)
                    if h<=1e-20 then break end
                    local step=math.max(-.25,math.min(.25,g/h))
                    local next=math.max(0,math.min(1,u-step))
                    if math.abs(next-u)<1e-12 then break end
                    u=next
                end
                consider(u)
            end
            if cooperate then cooperate() end
        end
    end
    assert(best,'curve_witness_missing')
    best.state=evaluate(best.curve.coefficients,best.u*.08,.08)
    best.identity=best.curve.identity;best.curve=nil
    best.evaluated_curves=evaluated
    return best
end
return M
