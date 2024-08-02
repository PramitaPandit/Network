//
// Generated file, do not edit! Created by opp_msgtool 6.0 from FeedbackThresholdMsg.msg.
//

#ifndef __FEEDBACKTHRESHOLDMSG_M_H
#define __FEEDBACKTHRESHOLDMSG_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// opp_msgtool version check
#define MSGC_VERSION 0x0600
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of opp_msgtool: 'make clean' should help.
#endif

class FeedbackThresholdMsg;
/**
 * Class generated from <tt>FeedbackThresholdMsg.msg:15</tt> by opp_msgtool.
 * <pre>
 * //
 * // This program is free software: you can redistribute it and/or modify
 * // it under the terms of the GNU Lesser General Public License as published by
 * // the Free Software Foundation, either version 3 of the License, or
 * // (at your option) any later version.
 * // 
 * // This program is distributed in the hope that it will be useful,
 * // but WITHOUT ANY WARRANTY; without even the implied warranty of
 * // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * // GNU Lesser General Public License for more details.
 * // 
 * // You should have received a copy of the GNU Lesser General Public License
 * // along with this program.  If not, see http://www.gnu.org/licenses/.
 * //
 * message FeedbackThresholdMsg
 * {
 *     double threshold;
 * }
 * </pre>
 */
class FeedbackThresholdMsg : public ::omnetpp::cMessage
{
  protected:
    double threshold = 0;

  private:
    void copy(const FeedbackThresholdMsg& other);

  protected:
    bool operator==(const FeedbackThresholdMsg&) = delete;

  public:
    FeedbackThresholdMsg(const char *name=nullptr, short kind=0);
    FeedbackThresholdMsg(const FeedbackThresholdMsg& other);
    virtual ~FeedbackThresholdMsg();
    FeedbackThresholdMsg& operator=(const FeedbackThresholdMsg& other);
    virtual FeedbackThresholdMsg *dup() const override {return new FeedbackThresholdMsg(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    virtual double getThreshold() const;
    virtual void setThreshold(double threshold);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const FeedbackThresholdMsg& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, FeedbackThresholdMsg& obj) {obj.parsimUnpack(b);}


namespace omnetpp {

template<> inline FeedbackThresholdMsg *fromAnyPtr(any_ptr ptr) { return check_and_cast<FeedbackThresholdMsg*>(ptr.get<cObject>()); }

}  // namespace omnetpp

#endif // ifndef __FEEDBACKTHRESHOLDMSG_M_H
