from pathlib import Path
p=Path(__file__).resolve().parent/'replay.cpp';s=p.read_text()
def one(a,b):
 global s
 assert s.count(a)==1,a
 s=s.replace(a,b)
one('Node::IntervalCenteringState interval; int wp=-1;', 'Node::IntervalCenteringState interval; interval.reference_length=in["cycles"][0]["rec_B_before"].get<double>(); int wp=-1;')
one('wp=pair[0];','wp=pair[0]; if(legacy){wp=c["rec_wp0"].get<int>();pair[0]=wp;pair[1]=c["rec_wp1"].get<int>();interval.reference_length=c["rec_B_before"].get<double>();}')
one('   Points prefix;bool used=', '   if(legacy)horizon=static_cast<int>(c["rec_preview"].size())-1;\n   Points prefix;bool used=')
one('   if(!used)previous.clear();', '''   if((legacy || out.empty()) && c["rec_preview"].size()==static_cast<size_t>(horizon+1)){
    prefix.clear();for(const auto&q:c["rec_preview"])prefix.push_back(body(V(q[0].get<double>(),q[1].get<double>())));used=true;
   }
   if(!used)previous.clear();''')
one('std::vector<std::pair<int,double>> runs={{0,0},{0,12}};', 'std::vector<std::pair<int,double>> runs={{0,0},{0,12},{1,0},{1,12}};')
p.write_text(s)
print('Carried planning memory and fixed recorded-reference counterfactual modes prepared.')
