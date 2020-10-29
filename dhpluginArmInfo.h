#ifndef DHPLUGINARMINFO_H
#define DHPLUGINARMINFO_H


#include <QObject>
#include "dhPluginInterface.h"

#include "dhArmature.h"
#include "dhMoCapSequence.h"
#include "dhFeaturePoint.h"
#include "dhSkeletalSubspaceDeformation.h"
//! [0]
class dhpluginArmInfo : public QObject, public dhPluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "dhPluginInterface")
    Q_INTERFACES(dhPluginInterface)
public:
    void initPlugin(void);
    QString echo(const QString &message);

    // Must be the same name as the top folder
    virtual QString packageName(void){ return "SamplePckg1"; }

    // 10 digits. "G"-"Z" can be used as characters of pakage ID.
    // Overwrap is not allowed.
    virtual QString packageID(void){ return "GHIJKLMOOP"; }

    // Tag name in "config.def", describing the serial code for this plugin
    virtual QString configSerialCode(void){ return "serial_test_pkg"; }

    // In case an user which has one of these IDs,
    // the user can use this plugin without serial code.
    virtual QStringList developerIDs(void){ return QStringList(); }
};
//! [0]




#include "IDHElement.h"
class dhArmOpe : public IDHElement
{
    Q_OBJECT
    DH_EXPOSE_TYPE
    DH_HIDDEN_CTOR_DTOR(dhArmOpe)
private:
    dhArmature *trgA;
    dhSkeletalSubspaceDeformation *trgSSD;
    dhFeaturePoints *trgFPs;
    dhMoCapSequence *trgMseq;
    QString fpname;

public slots:
    void writeBoneNum(dhArmature *arm);
    void saveArmInfo(dhArmature *arm, QString IFname);
    void saveArmInfoinMoCapSequence(dhArmature *arm, dhMoCapSequence *seq, QString OFname);
    void PlotGivenPointTrajectory(dhMoCapSequence *mocap, dhFeaturePoints *fp, QString FPName);
    void Extract_maxmin();

    virtual const bool	IsValid(void)const{ return (1); }
    virtual void Update(void){}//何もしない設定

    //element listで右クリックで出るアクションリスト
    QStringList ElementActionTitles();
    bool OnElementActionCalled(const QString& cmd);

    //element propertyの設定
    virtual void ConstructObjectProperty(void);
    virtual void OnObjectPropertyUpdated(const QString&);

};



#endif // DHPLUGINARMINFO_H

