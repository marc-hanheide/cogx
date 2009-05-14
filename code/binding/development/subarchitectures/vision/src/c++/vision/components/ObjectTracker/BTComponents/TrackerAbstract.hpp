#ifndef _TRACKER_ABSTRACT_HPP_
#define _TRACKER_ABSTRACT_HPP_

class TrackerAbstract 
{
protected:
    static bool m_saveRoiOption;

public:
    TrackerAbstract();
    virtual ~TrackerAbstract()=0;

    void SetSaveRoiOption(bool bOnOff) { m_saveRoiOption = bOnOff; }
    bool GetSaveRoiOption() { return m_saveRoiOption; } 
};

#endif
