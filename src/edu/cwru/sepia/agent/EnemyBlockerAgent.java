package edu.cwru.sepia.agent;

import edu.cwru.sepia.action.Action;
import edu.cwru.sepia.environment.model.history.DamageLog;
import edu.cwru.sepia.environment.model.history.History;
import edu.cwru.sepia.environment.model.state.*;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.*;

// Referenced classes of package edu.cwru.sepia.agent:
//            Agent

public class EnemyBlockerAgent extends Agent
{

    public EnemyBlockerAgent(int playernum)
    {
        super(playernum);
    }

    public Map initialStep(edu.cwru.sepia.environment.model.state.State.StateView stateView, edu.cwru.sepia.environment.model.history.History.HistoryView historyView)
    {
        return middleStep(stateView, historyView);
    }

    public Map middleStep(edu.cwru.sepia.environment.model.state.State.StateView stateView, edu.cwru.sepia.environment.model.history.History.HistoryView historyView)
    {
        Map actions = new HashMap();
        List unitIDs = stateView.getUnitIds(playernum);
        for(Iterator iterator = unitIDs.iterator(); iterator.hasNext();)
        {
            Integer unitID = (Integer)iterator.next();
            edu.cwru.sepia.environment.model.state.Unit.UnitView unit = stateView.getUnit(unitID.intValue());
            if(unit.getTemplateView().getName().toLowerCase().equals("footman"))
            {
                int turnNumber = stateView.getTurnNumber();
                List damageLogs = historyView.getDamageLogs(turnNumber - 1);
                for(Iterator iterator1 = damageLogs.iterator(); iterator1.hasNext();)
                {
                    DamageLog damageLog = (DamageLog)iterator1.next();
                    if(damageLog.getDefenderID() == unitID.intValue())
                    {
                        int enemyFootmanID = ((Integer)stateView.getUnitIds(0).get(0)).intValue();
                        actions.put(unitID, Action.createCompoundAttack(unitID.intValue(), enemyFootmanID));
                        return actions;
                    }
                }

                if(turnNumber < 2)
                    return actions;
                if(turnNumber < 33)
                    actions.put(unitID, Action.createCompoundMove(unitID.intValue(), 9, 12));
                else
                    actions.put(unitID, Action.createCompoundMove(unitID.intValue(), 11, 6));
                return actions;
            }
        }

        return actions;
    }

    public void terminalStep(edu.cwru.sepia.environment.model.state.State.StateView stateview, edu.cwru.sepia.environment.model.history.History.HistoryView historyview)
    {
    }

    public void savePlayerData(OutputStream outputstream)
    {
    }

    public void loadPlayerData(InputStream inputstream)
    {
    }
}