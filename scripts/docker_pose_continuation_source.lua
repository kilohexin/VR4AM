    local producer=worker(function()
        local seq=0;local max_span=0;local previous=nil
        print('CONT_REFERENCE',base.x,base.y,base.z)
        while not finish do
            local now=timestamp();local t=(now-start)/1000
            if not __BOUNDARY__ and t>=44 then break end
            assert(not previous or now-previous<100,'continuation_source_gap')
            local u=t<10 and t/10 or t<22 and 1 or t<32 and (32-t)/10 or 0
            local p={};for key,value in pairs(base) do p[key]=value end
            p.x=p.x+(__BOUNDARY__ and (t>=2 and .12 or 0) or .03*u)
            local sx,cx=math.sin(p.rx/2),math.cos(p.rx/2)
            local sy,cy=math.sin(p.ry/2),math.cos(p.ry/2)
            local sz,cz=math.sin(p.rz/2),math.cos(p.rz/2)
            local values={p.x*1e6,p.y*1e6,p.z*1e6,(sx*cy*cz-cx*sy*sz)*1e7,
                (cx*sy*cz+sx*cy*sz)*1e7,(cx*cy*sz-sx*sy*cz)*1e7,(cx*cy*cz+sx*sy*sz)*1e7}
            for i=1,7 do values[i]=math.floor(values[i]+.5) end
            local offset=240+(seq%2)*8;seq=seq+1
            lebai:set_signal(offset,0);lebai:set_signals(offset+1,values)
            assert(timestamp()-now+1<100,'continuation_publish_expired')
            lebai:set_signal(offset,(now%2097152)*1024+seq%1024)
            for i=1,6 do max_span=math.max(max_span,math.abs(tracker.q[i]-origin[i])) end
            previous=now;rt_sleep(20)
        end
        -- Silence, not a forged release stamp: receiver must retire on expiry.
        print('CONT_SOURCE_END',timestamp()-start,seq,max_span)
    end)
